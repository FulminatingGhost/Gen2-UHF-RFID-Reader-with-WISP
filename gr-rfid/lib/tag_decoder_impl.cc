/* -*- c++ -*- */
/*
* Copyright 2015 <Nikos Kargas (nkargas@isc.tuc.gr)>.
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING.  If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"

#define SHIFT_SIZE 3  // used in tag_detection

namespace gr
{
  namespace rfid
  {
    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {
      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr
      (new tag_decoder_impl(sample_rate,output_sizes));
    }

    /*
    * The private constructor
    */
    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
    : gr::block("tag_decoder",
    gr::io_signature::make(1, 1, sizeof(gr_complex)),
    gr::io_signature::makev(2, 2, output_sizes )),
    s_rate(sample_rate)
    {
      char_bits = (char *) malloc( sizeof(char) * 128);

      n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10,6);
      //GR_LOG_INFO(d_logger, "Number of samples of Tag bit : "<< n_samples_TAG_BIT);
    }

    /*
    * Our virtual destructor.
    */
    tag_decoder_impl::~tag_decoder_impl()
    {

    }

    void
    tag_decoder_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::tag_sync(std::vector<float> norm_in, int size)
    // This method searches the preamble and returns the start index of the tag data.
    // If the correlation value exceeds the threshold, it returns the start index of the tag data.
    // Else, it returns -1.
    // Threshold is an experimental value, so you might change this value within your environment.
    {
      int win_size = n_samples_TAG_BIT * TAG_PREAMBLE_BITS;
      float threshold = n_samples_TAG_BIT * 4;  // threshold verifing correlation value

      float max_corr = 0.0f;
      int max_index = 0;

      // compare all samples with sliding
      for(int i=0 ; i<size-win_size ; i++)  // i: start point
      {
        // calculate average_amp (threshold)
        float average_amp = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          average_amp += norm_in[i+j];
        average_amp /= win_size;

        // calculate normalize_factor
        float standard_deviation = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          standard_deviation += pow(norm_in[i+j] - average_amp, 2);
        standard_deviation /= win_size;
        standard_deviation = sqrt(standard_deviation);

        // calculate correlation value
        float corr_candidates[2] = {0.0f};
        for(int j=0 ; j<2*TAG_PREAMBLE_BITS ; j++)  // j: half_bit index of TAG_PREAMBLE
        {
          for(int k=0 ; k<(n_samples_TAG_BIT/2.0) ; k++)
          {
            for(int m=0 ; m<2 ; m++)  // m: index of TAG_PREAMBLE type
                corr_candidates[m] += TAG_PREAMBLE[m][j] * ((norm_in[i + j*(int)(n_samples_TAG_BIT/2.0) + k] - average_amp) / standard_deviation);
          }
        }

        // get max correlation value for ith start point
        float corr = 0.0f;
        for(int j=0 ; j<2 ; j++)
          if(corr_candidates[j] > corr) corr = corr_candidates[j];

        // compare with current max correlation value
        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      #ifdef DEBUG_MESSAGE
      {
        std::ofstream debug((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
        debug << "threshold= " << threshold << ", corr= " << max_corr << ", index=" << max_index << std::endl;
        debug << "\t\t\t\t\t** preamble samples **" << std::endl;
        for(int i=0 ; i<win_size ; i++)
          debug << norm_in[max_index+i] << " ";
        debug << std::endl << "\t\t\t\t\t** preamble samples **" << std::endl << std::endl << std::endl << std::endl;
        debug.close();
      }
      #endif

      // check if correlation value exceeds threshold
      if(max_corr > threshold) return max_index + win_size;
      else return -1;
    }

    int tag_decoder_impl::determine_first_mask_level(std::vector<float> norm_in, int index)
    // This method searches whether the first bit starts with low level or high level.
    // If the first bit starts with low level, it returns -1.
    // If the first bit starts with high level, it returns 0.
    // index: start point of "data bit", do not decrease half bit!
    {
      float max_max_corr = 0.0f;
      int max_max_index = -1;

      for(int k=0 ; k<2 ; k++)
      {
        float max_corr = 0.0f;
        int max_index = decode_single_bit(norm_in, index, k, &max_corr);

        if(max_corr > max_max_corr)
        {
          max_max_corr = max_corr;
          max_max_index = k;
        }
      }

      if(max_max_index == 0) max_max_index = -1;
      return max_max_index;
    }

    int tag_decoder_impl::decode_single_bit(std::vector<float> norm_in, int index, int mask_level, float* ret_corr)
    // This method decodes single bit and returns the decoded value and the correlation score.
    // index: start point of "data bit", do not decrease half bit!
    // mask_level: start level of "decoding bit". (-1)low level start, (1)high level start.
    {
      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low level start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high level start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing

      float max_corr = 0.0f;
      int max_index = -1;

      float average_amp = 0.0f;
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        average_amp += norm_in[index+j];
      average_amp /= (2*n_samples_TAG_BIT);

      // compare with two masks (0 or 1)
      for(int i=0 ; i<2 ; i++)
      {
        float corr = 0.0f;
        for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        {
          int idx;
          if(j < 0) idx = 0;                            // first section (trailing half bit of the previous bit)
          else if(j < (n_samples_TAG_BIT*0.5)) idx = 1; // second section (leading half bit of the data bit)
          else if(j < n_samples_TAG_BIT) idx = 2;       // third section (trailing half bit of the data bit)
          else idx = 3;                                 // forth section (leading half bit of the later bit)

          corr += masks[mask_level][i][idx] * (norm_in[index+j] - average_amp);
        }

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      (*ret_corr) = max_corr;
      return max_index;
    }

    std::vector<float> tag_decoder_impl::tag_detection(std::vector<float> norm_in, int index, int n_expected_bit)
    // This method decodes n_expected_bit of data by using previous methods, and returns the vector of the decoded data.
    // index: start point of "data bit", do not decrease half bit!
    {
      std::vector<float> decoded_bits;

      int mask_level = determine_first_mask_level(norm_in, index);
      int shift = 0;

      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;  // start point of decoding bit with shifting
        float max_corr = 0.0f;
        int max_index;
        int curr_shift;

        // shifting from idx-SHIFT_SIZE to idx+SHIFT_SIZE
        for(int j=0 ; j<(SHIFT_SIZE*2 + 1) ; j++)
        {
          float corr = 0.0f;
          int index = decode_single_bit(norm_in, idx+j-SHIFT_SIZE, mask_level, &corr);

          if(corr > max_corr)
          {
            max_corr = corr;
            max_index = index;
            curr_shift = j - SHIFT_SIZE;  // find the best current shift value
          }
        }

        #ifdef DEBUG_MESSAGE
        {
          std::ofstream debug((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "[" << i+1 << "th bit] corr=" << max_corr << ", curr_shift=" << curr_shift << ", shift=" << shift << ", decoded_bit=" << max_index;
          if(mask_level) debug << " (high start)" << std::endl;
          else debug << " (low start)" << std::endl;
          debug << "\t\t\t\t\t** shifted bit samples **" << std::endl;
          for(int j=idx-SHIFT_SIZE ; j<idx+n_samples_TAG_BIT+SHIFT_SIZE ; j++)
            debug << norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** shifted bit samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        if(max_index) mask_level *= -1; // change mask_level(start level of the next bit) when the decoded bit is 1

        decoded_bits.push_back(max_index);
        shift += curr_shift;  // update the shift value
      }

      return decoded_bits;
    }

    std::vector<int> tag_decoder_impl::cut_noise_sample(std::vector<float> in, const int size, const int data_len)
    {
      std::vector<int> output;

      int idx = 1;
      const float threshold = 0.002;
      float average = in[0];

      for(; idx<size ; idx++)
      {
        average += in[idx];
        average /= 2;
        if(std::abs(in[idx] - average) > threshold) break;
      }

      output.push_back(idx);  // start idx of the data sample

      idx += data_len*n_samples_TAG_BIT;
      average = in[idx];
      int count = 0;

      for(int i=1 ; idx+i<size ; i++)
      {
        average += in[idx+i];
        average /= 2;

        if(std::abs(in[idx+i] - average) > threshold)
        {
          count = 0;
          idx += i;
          i = 0;
          average = in[idx];
        }
        else count++;
        if(count >= 1.5 * n_samples_TAG_BIT) break;
      }

      output.push_back(idx-output[0]+1);  // size of the data sample
      return output;
    }

    // clustering algotirhm
    double tag_decoder_impl::IQ_distance(const gr_complex p1, const gr_complex p2)
    {
      return std::sqrt(std::pow((p1.real() - p2.real()), 2) + std::pow((p1.imag() - p2.imag()), 2));
    }

    std::vector<int> tag_decoder_impl::clustering_algorithm(std::vector<gr_complex> in, const int size)
    {
      std::vector<int> center_idx;

      std::vector<int> local_density;
      std::vector<double> local_distance;
      std::vector<int> min_distance_id;

      const double cutoff_distance = 0.0001;

      double max_local_density = 0;
      double max_local_distance = 0;

      std::ofstream parallel("parallel", std::ios::app);

      parallel << "\t\t\t\t\t** local density **" << std::endl;
      for(int i=0 ; i<size ; i++)
      {
        int current_local_density = -1;

        for(int j=0 ; j<size ; j++)
        {
          if(IQ_distance(in[i], in[j]) < cutoff_distance) current_local_density++;
        }

        parallel << current_local_density << " ";
        if(current_local_density > max_local_density) max_local_density = current_local_density;
        local_density.push_back(current_local_density);
      }
      parallel << std::endl << std::endl;

      parallel << "\t\t\t\t\t** local distance **" << std::endl;
      for(int i=0 ; i<size ; i++)
      {
        double min_distance = 1.7e308;
        int count = 0;int id;

        for(int j=0 ; j<size ; j++)
        {
          //if(i==j) continue;
          if(local_density[i] >= local_density[j]) continue;

          double distance = IQ_distance(in[i], in[j]);
          if(distance < min_distance) {min_distance = distance;id=j;}
          count++;
        }

        if(count == 0) min_distance = 0;
        parallel << min_distance << " ";
        if(min_distance > max_local_distance) max_local_distance = min_distance;
        local_distance.push_back(min_distance);min_distance_id.push_back(id);
      }
      parallel << std::endl << std::endl;

      max_local_density /= 10;
      max_local_distance /= 10;
      int count = 0;

      parallel << "\t\t\t\t\t** center idx **" << std::endl;
      for(int i=0 ; i<size ; i++)
      {
        if(local_density[i] > max_local_density && local_distance[i] > max_local_distance)
        {
          center_idx.push_back(i);
          parallel << i << " ";
          count++;
        }
      }
      parallel << std::endl << std::endl;

      if(count == 0) center_idx.push_back(-1);
      else
      {
        parallel << "\t\t\t\t\t** center (I) **" << std::endl;
        for(int i=0 ; i<center_idx.size() ; i++)
        {
          parallel << in[center_idx[i]].real() << " ";
        }
        parallel << std::endl << std::endl;

        parallel << "\t\t\t\t\t** center (Q) **" << std::endl;
        for(int i=0 ; i<center_idx.size() ; i++)
        {
          parallel << in[center_idx[i]].imag() << " ";
        }
        parallel << std::endl << std::endl;
      }
//std::cout << in[402].real() << " " << in[402].imag() << " " << in[1131].real() << " " << in[1131].imag() << " " << size << " ";
//std::cout <<"dis= " << IQ_distance(in[402], in[1131]) << " ";
std::cout<<std::endl;
    for(int i=0 ; i<center_idx.size() ; i++)
    {
      std::cout<<"i= " << center_idx[i] << " den= " << local_density[center_idx[i]] << " dis= " << local_distance[center_idx[i]] << " compare= " << min_distance_id[center_idx[i]] << " den= " << local_density[min_distance_id[center_idx[i]]] << std::endl;

      for(int j=0 ; j<center_idx.size() ; j++)
      {
        if(i==j) continue;


      }
    }

      parallel.close();
      return center_idx;
    }

    int
    tag_decoder_impl::general_work (int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const  gr_complex *) input_items[0];
      std::vector<float> norm_in;
      float *out = (float *) output_items[0];
      int consumed = 0;

      std::ofstream log;
      #ifdef DEBUG_MESSAGE
      std::ofstream debug;
      #endif

      // convert from complex value to float value
      for(int i=0 ; i<ninput_items[0] ; i++)
        norm_in.push_back(std::sqrt(std::norm(in[i])));

      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        std::vector<int> data_idx = cut_noise_sample(norm_in, ninput_items[0], TAG_PREAMBLE_BITS+RN16_BITS-1);

        std::vector<gr_complex> cut_in;
        std::vector<float> cut_norm_in;
        for(int i=data_idx[0] ; i<data_idx[0]+data_idx[1] ; i++)
        {
          cut_in.push_back(in[i]);
          cut_norm_in.push_back(norm_in[i]);
        }

        std::vector<int> center = clustering_algorithm(cut_in, data_idx[1]);

        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate **" << std::endl;
          for(int i=0 ; i<norm_in.size() ; i++)
            debug << norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;

          debug << "\t\t\t\t\t** (cut) samples from gate **" << std::endl;
          for(int i=0 ; i<cut_norm_in.size() ; i++)
            debug << cut_norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** (cut) samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate (I) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** samples from gate (Q) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;

          debug << "\t\t\t\t\t** (cut) samples from gate (I) **" << std::endl;
          for(int i=0 ; i<cut_in.size() ; i++)
            debug << cut_in[i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** samples from gate (Q) **" << std::endl;
          for(int i=0 ; i<cut_in.size() ; i++)
            debug << cut_in[i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // detect preamble
        int RN16_index = tag_sync(norm_in, ninput_items[0]);  //find where the tag data bits start
        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "\t\t\t\t\t** RN16 samples **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(RN16_BITS-1) ; i++)
            debug << norm_in[RN16_index+i] << " ";
          debug << std::endl << "\t\t\t\t\t** RN16 samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "\t\t\t\t\t** RN16 samples (I) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(RN16_BITS-1) ; i++)
            debug << in[RN16_index+i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** RN16 samples **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** RN16 samples (Q) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(RN16_BITS-1) ; i++)
            debug << in[RN16_index+i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** RN16 samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // process for GNU RADIO
        int written_sync = 0;
        for(int i=0 ; i<ninput_items[0] ; i++)
          written_sync++;
        produce(1, written_sync);

        // decode RN16
        if(RN16_index != -1)
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detected!" << std::endl;
          log.close();
          std::vector<float> RN16_bits = tag_detection(norm_in, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

          // write RN16_bits to the next block
          log.open("debug_message", std::ios::app);
          log << "│ RN16=";
          int written = 0;
          for(int i=0 ; i<RN16_bits.size() ; i++)
          {
            if(i % 4 == 0) std::cout << " ";
            log << RN16_bits[i];
            out[written++] = RN16_bits[i];
          }
          produce(0, written);

          // go to the next state
          log << std::endl << "├──────────────────────────────────────────────────" << std::endl;
          log.close();
          std::cout << "RN16 decoded | ";
          reader_state->gen2_logic_status = SEND_ACK;
        }
        else  // fail to detect preamble
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detection fail.." << std::endl;
          std::cout << "\t\t\t\t\tPreamble FAIL!!";

          reader_state->reader_stats.cur_slot_number++;
          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_inventory_round ++;
            reader_state->reader_stats.cur_slot_number = 1;

            log << "└──────────────────────────────────────────────────" << std::endl;
            if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
            {
              reader_state->reader_stats.cur_inventory_round--;
              reader_state-> status = TERMINATED;
              reader_state->decoder_status = DECODER_TERMINATED;
            }
            else
              reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            log << "├──────────────────────────────────────────────────" << std::endl;
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }
          log.close();
        }

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }

      // Processing only after n_samples_to_ungate are available and we need to decode an EPC
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {
        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << norm_in[i] << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
          debug << "\t\t\t\t\t** samples from gate (I) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** samples from gate (Q) **" << std::endl;
          for(int i=0 ; i<ninput_items[0] ; i++)
            debug << in[i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** samples from gate **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // detect preamble
        int EPC_index = tag_sync(norm_in, ninput_items[0]);
        #ifdef DEBUG_MESSAGE
        {
          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(), std::ios::app);
          debug << "\t\t\t\t\t** EPC samples **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << norm_in[EPC_index+i] << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();

          debug.open((debug_message+std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)+"_iq").c_str(), std::ios::app);
          debug << "\t\t\t\t\t** EPC samples (I) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << in[EPC_index+i].real() << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug << "\t\t\t\t\t** EPC samples (Q) **" << std::endl;
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
            debug << in[EPC_index+i].imag() << " ";
          debug << std::endl << "\t\t\t\t\t** EPC samples **" << std::endl << std::endl << std::endl << std::endl;
          debug.close();
        }
        #endif

        // process for GNU RADIO
        int written_sync = 0;
        for(int j=0 ; j<ninput_items[0] ; j++)
          written_sync++;
        produce(1, written_sync);

        // decode EPC
        if(EPC_index != -1)
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detected!" << std::endl;
          log.close();
          std::vector<float> EPC_bits = tag_detection(norm_in, EPC_index, EPC_BITS-1);  // EPC_BITS includes one dummy bit

          // convert EPC_bits from float to char in order to use Buettner's function
          log.open("debug_message", std::ios::app);
          log << "│ EPC=";
          for(int i=0 ; i<EPC_bits.size() ; i++)
          {
            if(i % 4 == 0) log << " ";
            log << EPC_bits[i];
            char_bits[i] = EPC_bits[i] + '0';
            if(i % 16 == 15) log << std::endl << "│     ";
          }
          log.close();

          // check CRC
          if(check_crc(char_bits, 128) == 1) // success to decode EPC
          {
            // calculate tag_id
            int tag_id = 0;
            for(int i=0 ; i<8 ; i++)
              tag_id += std::pow(2, 7-i) * EPC_bits[104+i];

            //GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << tag_id);
            log.open("debug_message", std::ios::app);
            log << "CRC check success! Tag ID= " << tag_id << std::endl;
            log.close();
            std::cout << "\t\t\t\t\t\t\t\t\t\tTag ID= " << tag_id;
            reader_state->reader_stats.n_epc_correct+=1;

            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(tag_id);
            if ( it != reader_state->reader_stats.tag_reads.end())
              it->second ++;
            else
              reader_state->reader_stats.tag_reads[tag_id]=1;
          }
          else
          {
            log.open("debug_message", std::ios::app);
            log << "│ CRC check fail.." << std::endl;
            log.close();
            std::cout << "\t\t\t\t\tCRC FAIL!!";
          }
        }
        else
        {
          log.open("debug_message", std::ios::app);
          log << "│ Preamble detection fail.." << std::endl;
          log.close();
          std::cout << "\t\t\t\t\tPreamble FAIL!!";
        }

        // After EPC message send a query rep or query
        log.open("debug_message", std::ios::app);
        reader_state->reader_stats.cur_slot_number++;
        if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
        {
          reader_state->reader_stats.cur_inventory_round ++;
          reader_state->reader_stats.cur_slot_number = 1;

          log << "└──────────────────────────────────────────────────" << std::endl;
          if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
          {
            reader_state->reader_stats.cur_inventory_round--;
            reader_state-> status = TERMINATED;
            reader_state->decoder_status = DECODER_TERMINATED;
          }
          else
            reader_state->gen2_logic_status = SEND_QUERY;
        }
        else
        {
          log << "├──────────────────────────────────────────────────" << std::endl;
          reader_state->gen2_logic_status = SEND_QUERY_REP;
        }
        log.close();

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
            data[i] = data[i] | mask;
          }
          mask = mask >> 1;
        }
      }
      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF;
      for (i=0; i < num_bytes - 2; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
          crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;

      if(rcvd_crc != crc_16)
      return -1;
      else
      return 1;
    }
  } /* namespace rfid */
} /* namespace gr */
