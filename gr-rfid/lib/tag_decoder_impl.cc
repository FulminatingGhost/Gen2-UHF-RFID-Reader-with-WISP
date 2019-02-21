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

    int tag_decoder_impl::tag_sync(float* norm_in, int size)
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

    int tag_decoder_impl::determine_first_mask_level(float* norm_in, int index)
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

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t\t[determine_first_mask_level] max_max_corr=" << max_max_corr;
      debug << "\t\t[determine_first_mask_level] max_max_corr=" << max_max_corr;
      if(max_max_index)
      {
        if(DEBUG_MESSAGE_TAG_DECODER) std::cout << ", high start" << std::endl;
        debug << ", high start" << std::endl;
      }
      else
      {
        if(DEBUG_MESSAGE_TAG_DECODER) std::cout << ", low start" << std::endl;
        debug << ", low start" << std::endl;
      }

      debug.close();
      if(max_max_index == 0) max_max_index = -1;
      return max_max_index;
    }

    int tag_decoder_impl::decode_single_bit(const gr_complex* in, int index, int mask_level, float* ret_corr)
    // index: start point of "tag data", do not decrease half bit!
    // mask_level: start level of "decoding bit", do not put start level of "previoud bit"! (-1)low start, (1)high start
    // corr: return max_corr
    {
      std::ofstream time("time/time", std::ios::app);
      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing

      float max_corr = 0.0f;
      int max_index = -1;
      int cut_off_samples = (int)(n_samples_TAG_BIT/2) * CUT_OFF;
      clock_t start, end;

      start = clock();
      
      float average_amp = 0.0f;
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        average_amp += in[index+j].real();
      average_amp /= (int)(2*n_samples_TAG_BIT);

      float average_abs_amp = 0.0f;
      for (int j=-(n_samples_TAG_BIT*0.5); j<(n_samples_TAG_BIT*1.5); j++)
          average_abs_amp += abs(in[index+j].real() - average_amp);
      average_abs_amp /= (int)(2*n_samples_TAG_BIT);

      for(int i=0 ; i<2 ; i++)
      {
        float corr = 0.0f;
        for(int j=0; j<4; j++)
        {
          int start_half_bit = index + (j-1)*(int)(n_samples_TAG_BIT/2);
          int end_half_bit = index + j*(int)(n_samples_TAG_BIT/2);
          
          for (int k=start_half_bit; k<end_half_bit; k++) {
            float scaled_amp = (in[k].real() - average_amp) / average_abs_amp;
            corr += masks[mask_level][i][j] * scaled_amp;
          }
        }

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }
      max_corr /= 2*(n_samples_TAG_BIT);
      (*ret_corr) = max_corr;

      end = clock();
      time << ((double)(end-start)/CLOCKS_PER_SEC) << std::endl;
      time.close();

      return max_index;
    }

    std::vector<float> tag_decoder_impl::tag_detection(const gr_complex* in, int index, int n_expected_bit)
    {
      std::vector<float> decoded_bits;

      std::ofstream debug(debug_file_path, std::ios::app);

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t[tag_decoder::tag_detection] Decoding " << n_expected_bit << " bit(s) of tag data.." << std::endl;
      debug << "\t[tag_decoder::tag_detection] Decoding " << n_expected_bit << " bit(s) of tag data.." << std::endl;

      int mask_level = determine_first_mask_level(in, index);
      int shift = 0;
      float threshold = 0.8f;
      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;
        float max_corr = 0.0f;
        int max_index;
        int curr_shift = 0;
       
        max_index = decode_single_bit(in, idx, mask_level, &max_corr);
        if (max_corr < threshold) {
            for (int j=0; j<(SHIFT_SIZE*2+1); j++) {
                float corr = 0.0f;
                int index = decode_single_bit(in, idx+j-SHIFT_SIZE, mask_level, &corr);
            
                if (corr > max_corr) {
                    max_corr = corr;
                    max_index = index;
                    curr_shift = j - SHIFT_SIZE;
                }
            }
        }
        shift += curr_shift;

        if(DEBUG_MESSAGE_TAG_DECODER_TAG_DETECTION) {
            std::cout << "\t\t[tag_detection " << i+1 << "th bit] max_corr=" << max_corr;
            std::cout << " shift=" << shift << ", decoded_bit=" << max_index;
        }
        debug << "\t\t[tag_detection " << i+1 << "th bit] max_corr=" << max_corr;
        debug << ", shift=" << shift << ", decoded_bit=" << max_index;

        if(mask_level)
        {
          if(DEBUG_MESSAGE_TAG_DECODER_TAG_DETECTION) 
              std::cout << " (high)" << std::endl;
          debug << " (high)" << std::endl;
        }
        else
        {
          if(DEBUG_MESSAGE_TAG_DECODER_TAG_DETECTION) 
              std::cout << " (low)" << std::endl;
          debug << " (low)" << std::endl;
        }

        if(max_index) mask_level *= -1; // change mask_level when the decoded bit is 1

        decoded_bits.push_back(max_index);
      }

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t[tag_detection] decoded_bits=\t";
      debug << "\t[tag_detection] decoded_bits=\n";

      for(int i=0 ; i<n_expected_bit ; i++)
      {
        if(DEBUG_MESSAGE_TAG_DECODER) std::cout << decoded_bits[i];
        debug << decoded_bits[i];

        if(i % 4 == 3)
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << " ";
          debug << " ";
        }
        if(i % 32 == 31)
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << std::endl << "\t\t\t\t\t";
          debug << std::endl << "\t\t\t\t\t";
        }
      }

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << std::endl;
      debug << std::endl;

      debug.close();
      return decoded_bits;
    }

    int
    tag_decoder_impl::general_work (int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const  gr_complex *) input_items[0];
      float *out = (float *) output_items[0];

      int written = 0, consumed = 0;

      std::vector<float> EPC_samples_real;
      std::vector<gr_complex> EPC_samples_complex;

      int number_of_half_bits = 0;
      int number_of_points = 0;


      std::ofstream debug(debug_file_path, std::ios::app);

      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        if(DEBUG_MESSAGE_TAG_DECODER)
        {
          std::cout << "[tag_decoder] Ready to decode RN16.." << std::endl;
          std::cout << "\tn_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
        }
        debug << "[tag_decoder] Ready to decode RN16.." << std::endl;
        debug << "\tn_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;

        // detect preamble
        int RN16_index = tag_sync(in, ninput_items[0]);  //find where the tag data bits start

        // process for GNU RADIO
        int written_sync = 0;
        for(int j=0 ; j<ninput_items[0] ; j++)
          written_sync++;
        produce(1, written_sync);

        // decode RN16
        if(RN16_index == -1)  // fail to detect preamble
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] Fail to detect preamble!" << std::endl;
          debug << "[tag_decoder] Fail to detect preamble!" << std::endl;

          reader_state->reader_stats.cur_slot_number++;
          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_slot_number = 1;
            reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());

            reader_state->reader_stats.cur_inventory_round += 1;

            //if (P_DOWN == true)
            //  reader_state->gen2_logic_status = POWER_DOWN;
            //else

            if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\tSlot is full.. Go to new inventory round.." << std::endl;
            debug << "\tSlot is full.. Go to new inventory round.." << std::endl;
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }

          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << std::endl;
          debug << std::endl;
        }
        else
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] Decoding RN16.." << std::endl;
          debug << "[tag_decoder] Decoding RN16.." << std::endl;

          std::vector<float> RN16_bits = tag_detection(in, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

          // write RN16_bits to the next block
          int written = 0;
          for(int i=0 ; i<RN16_bits.size() ; i++)
            out[written++] = RN16_bits[i];
          produce(0, written);

          // go to the next state
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] RN16 decoded.." << std::endl << std::endl;
          debug << "[tag_decoder] RN16 decoded.." << std::endl << std::endl;
          reader_state->gen2_logic_status = SEND_ACK;
        }

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }
      // Processing only after n_samples_to_ungate are available and we need to decode an EPC
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {
        if(DEBUG_MESSAGE_TAG_DECODER)
        {
          std::cout << "[tag_decoder] Ready to decode EPC.." << std::endl;
          std::cout << "\tn_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
        }
        debug << "[tag_decoder] Ready to decode EPC.." << std::endl;
        debug << "\tn_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;

        // detect preamble
        int EPC_index = tag_sync(in, ninput_items[0]);

        // process for GNU RADIO
        int written_sync = 0;
        for(int j=0 ; j<ninput_items[0] ; j++)
          written_sync++;
        produce(1, written_sync);

        if(EPC_index == -1)  // fail to detect preamble
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] Fail to detect preamble!" << std::endl;
          debug << "[tag_decoder] Fail to detect preamble!" << std::endl;

          for(int i=0 ; i<EPC_BITS-1 ; i++)
            char_bits[i] = 0;
        }
        else
        {
          // decode EPC
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] Decoding EPC.." << std::endl;
          debug << "[tag_decoder] Decoding EPC.." << std::endl;

          std::vector<float> EPC_bits = tag_detection(in, EPC_index, EPC_BITS-1);

          // convert EPC_bits from float to char in order to use Buettner's function
          for(int i=0 ; i<EPC_BITS-1 ; i++)
          {
            if(EPC_bits[i]) char_bits[i] = '1';
            else char_bits[i] = '0';
          }
        }

        // After EPC message send a query rep or query
        reader_state->reader_stats.cur_slot_number++;

        // check CRC
        if(check_crc(char_bits, 128) == 1) // success to decode EPC
        {
          // calculate tag_id
          int tag_id = 0;
          for(int i=0 ; i<8 ; i++)
          {
            tag_id += std::pow(2, 7-i) * (char_bits[104+i] - '0');
          }

          GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << tag_id);
          std::cout << "                                                                EPC CORRECTLY DECODED TAG ID : " << tag_id << std::endl;
          debug << "                                                                EPC CORRECTLY DECODED TAG ID : " << tag_id << std::endl;

          // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
          std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(tag_id);
          if ( it != reader_state->reader_stats.tag_reads.end())
          {
            it->second ++;
          }
          else
          {
            reader_state->reader_stats.tag_reads[tag_id]=1;
          }

          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_slot_number = 1;
            reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());

            reader_state->reader_stats.cur_inventory_round+=1;
            //if (P_DOWN == true)
            //  reader_state->gen2_logic_status = POWER_DOWN;
            //else
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }

          reader_state->reader_stats.n_epc_correct+=1;

        }
        else  // fail to decode EPC
        {
          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_slot_number = 1;
            reader_state->reader_stats.cur_inventory_round+=1;
            //if (P_DOWN == true)
            //  reader_state->gen2_logic_status = POWER_DOWN;
            //else
            //  reader_state->gen2_logic_status = SEND_NAK_Q;
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            //reader_state->gen2_logic_status = SEND_NAK_QR;
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }

          GR_LOG_INFO(d_debug_logger, "EPC FAIL TO DECODE");
        }

        consumed = reader_state->n_samples_to_ungate;
      }
      consume_each(consumed);
      debug.close();
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
