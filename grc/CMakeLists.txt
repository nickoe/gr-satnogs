# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA

add_subdirectory(satellites)

list(APPEND debug_blocks
    satnogs_morse_debug_source.xml
    satnogs_debug_msg_source.xml
    satnogs_debug_msg_source_raw.xml
    satnogs_leo_channel.xml
    satnogs_cw_encoder.xml 
)

list(APPEND enabled_blocks
    satnogs_block_tree.xml
    satnogs_ax25_decoder_bm.xml
    satnogs_ax25_encoder_mb.xml
    satnogs_ccsds_rs_decoder_mm.xml
    satnogs_decoder_8b10b.xml
    satnogs_fox_telem_mm.xml
    satnogs_frame_file_sink.xml
    satnogs_iq_sink.xml
    satnogs_lrpt_decoder.xml 
    satnogs_lrpt_sync.xml 
    satnogs_morse_decoder.xml
    satnogs_multi_format_msg_sink.xml
    satnogs_ogg_encoder.xml 
    satnogs_cw_to_symbol.xml
    satnogs_udp_msg_source.xml
    satnogs_tcp_rigctl_msg_source.xml
    satnogs_doppler_correction_cc.xml
    satnogs_udp_msg_sink.xml
    satnogs_coarse_doppler_correction_cc.xml
    satnogs_waterfall_sink.xml
    satnogs_quad_demod_filter_ff.xml
    satnogs_ogg_source.xml
    satnogs_noaa_apt_sink.xml
    satnogs_whitening.xml
    satnogs_frame_acquisition.xml
)

if(${INCLUDE_DEBUG_BLOCKS})
	list(APPEND enabled_blocks ${debug_blocks})
endif()
install(FILES
    ${enabled_blocks}
    DESTINATION share/gnuradio/grc/blocks
)
