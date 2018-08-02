/* -*- c++ -*- */
/*
 * gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2018, Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "lrpt_decoder_impl.h"
#include <satnogs/log.h>
extern "C" {
  #include <fec.h>
}

namespace gr
{
namespace satnogs
{

lrpt_decoder::sptr
lrpt_decoder::make ()
{
  return gnuradio::get_initial_sptr (new lrpt_decoder_impl ());
}

/*
 * The private constructor
 */
lrpt_decoder_impl::lrpt_decoder_impl()
: gr::block("lrpt_decoder",
    gr::io_signature::make(0, 0, 0),
    gr::io_signature::make(0, 0, 0)),
    d_cadu_len(1020 + 4),
    d_coded_cadu_len(1020 * 2 + 4*2),
    d_conv_deinterl(36, 2048)
{

  message_port_register_in(pmt::mp("cadu"));
  message_port_register_out(pmt::mp("frame"));

  set_msg_handler (
      pmt::mp ("cadu"),
      boost::bind (&lrpt_decoder_impl::decode, this, _1));

  d_vt = create_viterbi27(d_cadu_len * 8);
  if(!d_vt) {
    throw std::runtime_error("lrpt_decoder: Failed to init Viterbi decoder");
  }
  int polys[2] = {0x79, 0x5b};
  set_viterbi27_polynomial(polys);

  d_cadu = new uint8_t[d_cadu_len];
  d_coded_cadu_syms = new uint8_t[d_coded_cadu_len * 8];

}

/*
 * Our virtual destructor.
 */
lrpt_decoder_impl::~lrpt_decoder_impl ()
{

  delete [] d_cadu;
  delete [] d_coded_cadu_syms;
}

void
lrpt_decoder_impl::decode (pmt::pmt_t m)
{
  const uint8_t *coded_cadu = (const uint8_t *)pmt::blob_data(m);
  if(pmt::blob_length(m) != d_coded_cadu_len) {
    LOG_ERROR("Wrong CADU size");
    return;
  }

  init_viterbi27(d_vt, 0);

  for(size_t i = 0; i < d_coded_cadu_len; i++) {
    d_coded_cadu_syms[i * 8] = ~(255 + (coded_cadu[i] >> 7));
    d_coded_cadu_syms[i * 8 + 1] = ~(255 + (coded_cadu[i] >> 6) & 0x1);
    d_coded_cadu_syms[i * 8 + 2] = ~(255 + (coded_cadu[i] >> 5) & 0x1);
    d_coded_cadu_syms[i * 8 + 3] = ~(255 + (coded_cadu[i] >> 4) & 0x1);
    d_coded_cadu_syms[i * 8 + 4] = ~(255 + (coded_cadu[i] >> 3) & 0x1);
    d_coded_cadu_syms[i * 8 + 5] = ~(255 + (coded_cadu[i] >> 2) & 0x1);
    d_coded_cadu_syms[i * 8 + 6] = ~(255 + (coded_cadu[i] >> 1) & 0x1);
    d_coded_cadu_syms[i * 8 + 7] = ~(255 + (coded_cadu[i] & 0x1));
  }
  update_viterbi27_blk(d_vt, d_coded_cadu_syms, d_cadu_len * 8);
  chainback_viterbi27(d_vt, d_cadu, d_cadu_len * 8, 0);
  message_port_pub(pmt::mp("frame"), pmt::make_blob(d_cadu, d_cadu_len));
}



} /* namespace satnogs */
} /* namespace gr */

