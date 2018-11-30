/* -*- c++ -*- */
/*
 * gr-satnogs: SatNOGS GNU Radio Out-Of-Tree Module
 *
 *  Copyright (C) 2016, 2017, 2018
 *  Libre Space Foundation <http://librespacefoundation.org/>
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
#include <satnogs/golay24.h>

#include <satnogs/utils.h>

namespace gr
{
namespace satnogs
{

golay24::golay24 ()
{

  d_H = { 0x7ff, 0xee2, 0xdc5, 0xb8b, 0xf16, 0xe2d, 0xc5b, 0x8b7, 0x96e, 0xadc,
      0xdb8, 0xb71 };
  d_X = {0x800, 0x400, 0x200, 0x100, 0x080, 0x040, 0x020, 0x010, 0x008, 0x004,
      0x002, 0x001};
}

golay24::~golay24 ()
{
}

/**
 * Encodes a 12-bit message
 * @param in the input 12-bit message. The message should be placed at the
 * 12 LS bits
 * @return the coded 24-bit message. The message is placed at the 24 LS bits
 */
uint32_t
golay24::encode12 (uint16_t in)
{
  uint32_t c[2] =
    { 0x0, 0x0 };
  c[0] = in & 0xFFF;
  for (size_t i = 0; i < 12; i++) {
    uint32_t tmp = 0;
    for (size_t j = 0; j < 12; j++) {
      tmp ^= (((c[0] & d_H[i]) >> j) & 0x01);
    }
    c[1] = (c[1] << 1) ^ tmp;
  }
  return ((c[0] & 0xFFF) << 12) | (c[1] & 0xFFF);
}

static inline uint32_t
ham_dist(uint32_t x)
{
  return bit_count(x & 0xFFF);
}

/**
 * Decodes a single Golay (24, 12, 8) codeword
 *
 * @param out the 12-bit decoded message. The message is placed at the 12 LS bits
 * @param in the coded 24 bit code word. The message should be placed at the
 * 24 LS bits
 * @return true if the decoding was successful, false in case the error correction
 * could not be performed
 */
bool
golay24::decode24(uint16_t *out, const uint32_t in)
{
  uint32_t e[2] = {0x0, 0x0};
  uint32_t r[2] = {0x0, 0x0};
  uint32_t c[2] = {0x0, 0x0};
  uint32_t syndrome = 0x0;
  bool found = false;
  uint32_t col = 0;
  uint32_t q = 0;

  r[0] = (in >> 12) & 0xFFF;
  r[1] = in & 0xFFF;

  for(size_t j = 0; j < 12; j++) {
    uint32_t tmp = 0x0;
    for(size_t i = 0; i < 12; i++) {
      tmp ^= (((d_X[j] & r[0]) >> i) & 0x1);
    }

    for(size_t i = 0; i < 12; i++) {
      tmp ^= (((d_H[j] & r[1]) >> i) & 0x1);
    }
    syndrome = (syndrome << 1) ^ tmp;
  }

  if(ham_dist(syndrome) <= 3) {
    e[0] = syndrome;
    e[1] = 0x0;
  }
  else{
    do {
      if (ham_dist (syndrome ^ d_H[col]) <= 2) {
        e[0] = syndrome ^ d_H[col];
        e[1] = d_X[col];
        found = 1;
      }
      col++;
    }
    while ((col < 12) && !found);

    if ((col == 12) && !found) {
      for (size_t j = 0; j < 12; j++) {
        uint32_t tmp = 0x0;
        for (size_t i = 0; i < 12; i++) {
          tmp ^= (((d_H[j] & syndrome) >> i) & 0x1);
        }
        q = (q << 1) ^ tmp;
      }
    }

    if (ham_dist (q) <= 3) {
      e[0] = 0;
      e[1] = q;
    }
    else {
      col = 0;
      found = 0;
      do {
        if (ham_dist (q ^ d_H[col]) <= 2) {
          e[0] = d_X[col];
          e[1] = q ^ d_H[col];
          found = 1;
        }
        col++;
      }
      while ((col < 12) && !found);

      if (col == 12 && !found) {
        return false;
      }
    }
  }
  c[0] = r[0] ^ e[0];
  c[1] = r[1] ^ e[1];
  /* Return the message part only, not the parity */
  *out = c[0];
  return true;
}

} /* namespace satnogs */
} /* namespace gr */

