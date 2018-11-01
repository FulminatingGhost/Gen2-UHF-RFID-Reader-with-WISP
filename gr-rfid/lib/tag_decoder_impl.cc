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

#define DEBUG_MESSAGE_TAG_DECODER 1
#define DEBUG_MESSAGE_TAG_DECODER_DECODE_SINGLE_BIT 1
#define DEBUG_MESSAGE_TAG_DECODER_TAG_DETECTION 1
#define SHIFT_SIZE 0  // used in tag_detection

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
      GR_LOG_INFO(d_logger, "Number of samples of Tag bit : "<< n_samples_TAG_BIT);
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

    int tag_decoder_impl::tag_sync(const gr_complex* in, int size)
    {
      // This method searches the preamble and returns the start index of the tag data.
      // If the correlation value exceeds the threshold, it returns the start index of the tag data.
      // Else, it returns -1.
      // Threshold is an experimental value, so you might change this value within your environment.

      int win_size = n_samples_TAG_BIT * TAG_PREAMBLE_BITS;
      float threshold = 0.01f;  // threshold verifing correlation value

      float max_corr = 0.0f;
      int max_index = 0;

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t[tag_decoder::tag_sync] Detecting preamble.." << std::endl;

      // compare all samples with sliding
      for(int i=0 ; i<size-win_size ; i++)  // i: start point
      {
        // calculate average_amp (threshold)
        float average_amp = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          average_amp += in[i].real();
        average_amp /= win_size;

        // calculate correlation value
        float corr_candidates[2] = {0.0f};
        for(int j=0 ; j<2*TAG_PREAMBLE_BITS ; j++)  // j: half_bit index of TAG_PREAMBLE
        {
          for(int k=0 ; k<(n_samples_TAG_BIT/2.0) ; k++)
          {
            for(int m=0 ; m<2 ; m++)  // m: index of TAG_PREAMBLE type
                corr_candidates[m] += TAG_PREAMBLE[m][j] * (in[i + j*(int)(n_samples_TAG_BIT/2.0) + k].real() - average_amp);
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

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t\t[tag_sync] max_corr= " << max_corr << "\tmax_index= " << max_index << std::endl;

      // check if correlation value exceeds threshold
      if(max_corr > threshold)
      {
        if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t\t[tag_sync] Preamble successfully detected.." << std::endl;
        return max_index + win_size;
      }
      else
      {
        if(DEBUG_MESSAGE_TAG_DECODER)
        {
          std::cout << "\t\t[tag_sync] Preamble detection fail.." << std::endl;
          std::cout << "\t\t\tCheck whether if the threshold value is too high!" << std::endl;
          std::cout << "\t\t\tCurrent threshold= " << threshold << std::endl;
        }
        return -1;
      }
    }

    int tag_decoder_impl::determine_first_mask_level(const gr_complex* in, int index)
    // index: start point of "tag data", do not decrease half bit!
    {
      float max_max_corr = 0.0f;
      int max_max_index = -1;

      for(int k=0 ; k<2 ; k++)
      {
        float max_corr = 0.0f;
        int max_index = decode_single_bit(in, index, k, &max_corr);

        if(max_corr > max_max_corr)
        {
          max_max_corr = max_corr;
          max_max_index = k;
        }
      }

      if(DEBUG_MESSAGE_TAG_DECODER)
      {
        std::cout << "\t\t[determine_first_mask_level] max_max_corr=" << max_max_corr;
        if(max_max_index) std::cout << ", high start" << std::endl;
        else std::cout << ", low start" << std::endl;
      }

      return max_max_index;
    }

    int tag_decoder_impl::decode_single_bit(const gr_complex* in, int index, int mask_level, float* ret_corr)
    // index: start point of "tag data", do not decrease half bit!
    // mask_level: start level of "decoding bit", do not put start level of "previoud bit"! (0)low start, (1)high start
    // corr: return max_corr
    {
      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing

      float max_corr = 0.0f;
      int max_index = -1;

      for(int i=0 ; i<2 ; i++)
      {
        float average_amp = 0.0f;
        for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
          average_amp += in[j].real();
        average_amp /= (2*n_samples_TAG_BIT);

        float corr = 0.0f;
        for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        {
          int idx;
          if(j < 0) idx = 0;
          else if(j < (n_samples_TAG_BIT*0.5)) idx = 1;
          else if(j < n_samples_TAG_BIT) idx = 2;
          else idx = 3;

          corr += masks[mask_level][i][idx] * (in[index+j].real() - average_amp);
        }

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

      if(DEBUG_MESSAGE_TAG_DECODER_DECODE_SINGLE_BIT)
      {
        std::cout << "\t\t\t[decode_single_bit] max_corr=" << max_corr << ", decoded bit=" << max_index;
        if(mask_level) std::cout << " (high start)" << std::endl;
        else std::cout << " (low start)" << std::endl;
      }

      (*ret_corr) = max_corr;
      return max_index;
    }

    std::vector<float> tag_decoder_impl::tag_detection(const gr_complex* in, int index, int n_expected_bit)
    {
      std::vector<float> decoded_bits;

      if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "\t[tag_decoder::tag_detection] Decoding " << n_expected_bit << " bit(s) of tag data.." << std::endl;

      FILE* file = fopen("rn16", "w");

      int mask_level = determine_first_mask_level(in, index);
      int shift = 0;
      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;
        float max_corr = 0.0f;
        int max_index;
        int curr_shift;

        for(int j=0 ; j<(SHIFT_SIZE*2 + 1) ; j++)
        {
          float corr = 0.0f;
          int index = decode_single_bit(in, idx+j-SHIFT_SIZE, mask_level, &corr);

          if(corr > max_corr)
          {
            max_corr = corr;
            max_index = index;
            curr_shift = j - SHIFT_SIZE;
          }
        }

        decoded_bits.push_back(max_index);
        shift += curr_shift;

        fprintf(file, "%d\n", i);
        for(int j=-(n_samples_TAG_BIT*0.5) ; j<1.5*n_samples_TAG_BIT ; j++)
          fprintf(file, "%f ", in[idx+j].real());
        fprintf(file, "\n\n");

        if(DEBUG_MESSAGE_TAG_DECODER_TAG_DETECTION)
        {
          std::cout << "\t\t[tag_detection] max_corr=" << max_corr << ", curr_shift=" << curr_shift << ", shift=" << shift << ", decoded_bit=" << max_index;

          if(mask_level) std::cout << " (high start)" << std::endl;
          else std::cout << " (low start)" << std::endl;
        }

        if(max_index) mask_level *= -1; // change mask_level when the decoded bit is 1
      }
      fclose(file);
      if(DEBUG_MESSAGE_TAG_DECODER)
      {
        std::cout << "\t\t[tag_detection] decoded_bits=";
        for(int i=0 ; i<n_expected_bit ; i++)
        {
          if(i % 4 == 0) std::cout << " ";
          std::cout << decoded_bits[i];
        }
        std::cout << std::endl;
      }

      return decoded_bits;
    }

    std::vector<float> tag_decoder_impl::bit_decoding(
      std::vector<gr_complex> &samples_complex, // samples_complex must start half bits less than real signal start point
      int                     n_expected_bit,
      int                     index)  // index not using.. need to delete
    {
      std::vector<float> tag_bits;

      const float masks[4][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {-1, 1, -1, 1}, // 0
        {1, -1, 1, -1}, // 0
        {1, -1, -1, 1}, // 1
        {-1, 1, 1, -1}  // 1
      };
      int start, end;
      int shift_cum=0;

      preamble_fp = fopen(("decode_data/"+std::to_string(n_expected_bit)+"_raw_"+std::to_string(reader_state->reader_stats.cur_inventory_round-1)).c_str(), "w");
      for(int i=0 ; i<(n_expected_bit+1)*n_samples_TAG_BIT ; i++)
        fprintf(preamble_fp, "%f ", samples_complex[i].real());
      fclose(preamble_fp);

      preamble_fp = fopen(("decode_data/"+std::to_string(n_expected_bit)+"_"+std::to_string(reader_state->reader_stats.cur_inventory_round-1)).c_str(), "w");
      if(!preamble_fp) std::cout << "(tag_decoding_impl.cpp::bit_decoding)File open error!" << std::endl;

      //decode bit every round
      for (int i = 0; i < n_expected_bit; i++) {
        float corr[SHIFT_SIZE*2+1][4] = {0.0f,};  // store correlation scores for each shift
        //std::cout << i << std::endl;
        fprintf(preamble_fp,"%d ",i);
        start = (int)(i*n_samples_TAG_BIT)+shift_cum;
        end = start + (int)(2*n_samples_TAG_BIT);

        float average_amp = 0;

        // calculate average_amp
        for(int j = start;j < end; j++)
          average_amp += samples_complex[j].real();
        average_amp = average_amp/(int)(n_samples_TAG_BIT * 2);

        //calculating correlation values
        for (int j = 0; j < 4; j++) { // compare with 4 masks
          for (int k = start+SHIFT_SIZE; k < end-SHIFT_SIZE; k++) { // cut SHIFT_SIZE samples at each boundary
            int devi = 0;
            int position = k-start;

            // get location
            if(position<(n_samples_TAG_BIT*0.5))  //first quarter
              devi = 0;
            else if(position<(n_samples_TAG_BIT)) //second quarter
              devi = 1;
            else if(position<(n_samples_TAG_BIT*1.5)) //third quarter
              devi = 2;
            else  //last quarter
              devi = 3;

            if(j==0){
              fprintf(preamble_fp, ", ");
              fprintf(preamble_fp, "%f", samples_complex[k].real());
              //std::cout << samples_complex[k].real() << std::endl;
            }
            for(int l=-SHIFT_SIZE;l<=SHIFT_SIZE;l++){ // iterate shift cases
              corr[l+SHIFT_SIZE][j] += masks[j][devi] * (std::real(samples_complex[k+l])-average_amp);  //calculate
              //
            }
          }
        }
        fprintf(preamble_fp, "\n%d ",i);

        int maxidx[SHIFT_SIZE*2+1] = {0,};
        int secondidx[SHIFT_SIZE*2+1] = {0,};

        //find the most maximum correlation values
        for (int i = 0; i < 4; i++) {
          for(int j=0; j<=SHIFT_SIZE*2; j++){
            if (corr[j][i] > corr[j][maxidx[j]]){
              secondidx[j] = maxidx[j];
              maxidx[j] = i;
            }else if(corr[j][i] > corr[j][secondidx[j]])
            secondidx[j] = i;
          }
        } // why get secondidx??
        // now maxidx[j] stores 0~3 value which indicates max_corr mask
        // maxidx 0, 1 --> value 0
        // maxidx 2, 3 --> value 1

        for(int i = 0;i<=SHIFT_SIZE*2;i++){
          fprintf(preamble_fp,", %d",maxidx[i]);
        }
        fprintf(preamble_fp, "\n%d ",i);
        for(int i = 0;i<=SHIFT_SIZE*2;i++){
          fprintf(preamble_fp,", %f",corr[i][maxidx[i]]);
        }

        int shift = 0;  //actual value is shift-2
        float diff[SHIFT_SIZE*2+1];


        //find out whether shift or not
        for (int i = 0; i<SHIFT_SIZE*2+1;i++){
          if(corr[i][maxidx[i]] > corr[shift][maxidx[shift]])
          shift = i;
        }
        fprintf(preamble_fp, "\n");

        //std::cout<<shift-SHIFT_SIZE<<" ";

        shift_cum += (shift-SHIFT_SIZE);

        //based on maximum correlation value, decode the tag bits
        if (maxidx[shift] <= 1){
          tag_bits.push_back(0);
        }
        else{
          tag_bits.push_back(1);
        }
      }

      fclose(preamble_fp);
      //std::cout<<std::endl<<"shift cum : "<<shift_cum<<std::endl;

      return tag_bits;
    }


    int
    tag_decoder_impl::general_work (int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const  gr_complex *) input_items[0];
      float *out = (float *) output_items[0];

      int written_sync =0;
      int written = 0, consumed = 0;
      int  EPC_index;

      std::vector<float> RN16_samples_real;
      std::vector<float> EPC_samples_real;

      std::vector<gr_complex> RN16_samples_complex;
      std::vector<gr_complex> EPC_samples_complex;

      std::vector<float> RN16_bits;
      int number_of_half_bits = 0;
      int number_of_points = 0;

      std::vector<float> EPC_bits;

      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        if(DEBUG_MESSAGE_TAG_DECODER)
        {
          std::cout << "[tag_decoder] Ready to decode RN16.." << std::endl;
          std::cout << "\tn_samples_to_ungate= " << reader_state->n_samples_to_ungate << ", ninput_items[0]= " << ninput_items[0] << std::endl;
        }

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
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }

          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << std::endl;
        }
        else
        {
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] Decoding RN16.." << std::endl;

          std::vector<float> RN16_bits = tag_detection(in, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

          // write RN16_bits to the next block
          int written = 0;
          for(int i=0 ; i<RN16_bits.size() ; i++)
            out[written++] = RN16_bits[i];
          produce(0, written);

          // go to next state
          if(DEBUG_MESSAGE_TAG_DECODER) std::cout << "[tag_decoder] RN16 decoded.." << std::endl << std::endl;
          while(1) ;  //temp for debugging [!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!REMOVE HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!]
          reader_state->gen2_logic_status = SEND_ACK;
        }

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }
      // Processing only after n_samples_to_ungate are available and we need to decode an EPC
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {

        //After EPC message send a query rep or query
        reader_state->reader_stats.cur_slot_number++;

        EPC_index = tag_sync(in,ninput_items[0]);
        for (int j = 0 ; j < (EPC_BITS + 2)*n_samples_TAG_BIT ; j++ )
        {
          EPC_samples_complex.push_back(in[EPC_index+j]);
        }

        for (int j = 0; j < ninput_items[0] ; j ++ )
        {
          //out_2[written_sync].real() = in[j].real();
          written_sync ++;
        }
        produce(1,written_sync);


        EPC_bits = bit_decoding(EPC_samples_complex,EPC_BITS-1,0);
        std::cout << "                                                                  EPC detect?" << std::endl;
        for(int i=0 ; i<128 ; i++)
        {
          if(i%4==0) std::cout << " ";
          if(i%16==0) std::cout << "\t";
          if(i%32==0) std::cout << std::endl;
          std::cout << EPC_bits[i];
        }

        if (EPC_bits.size() == EPC_BITS - 1)
        {
          std::cout << "                                                                EPC detected!!!!!!!!" << std::endl;
          // float to char -> use Buettner's function
          for (int i =0; i < 128; i ++)
          {
            if (EPC_bits[i] == 0)
            char_bits[i] = '0';
            else
            char_bits[i] = '1';
          }
          if(check_crc(char_bits,128) == 1)
          {
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

            int result = 0;
            for(int i = 0 ; i < 8 ; ++i)
            {
              result += std::pow(2,7-i) * EPC_bits[104+i] ;
            }
            GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << result);
            std::cout << "                                                                EPC CORRECTLY DECODED TAG ID : " << result << std::endl;
            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(result);
            if ( it != reader_state->reader_stats.tag_reads.end())
            {
              it->second ++;
            }
            else
            {
              reader_state->reader_stats.tag_reads[result]=1;
            }
          }
          else
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
        }
        else
        {
          GR_LOG_EMERG(d_debug_logger, "CHECK ME");
        }
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
