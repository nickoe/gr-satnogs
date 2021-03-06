#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: BPSK Decoder
# Author: Patrick Dohmen, DL4PD
# Description: A BPSK decoder block for gr-satnogs
# Generated: Tue Mar 27 17:05:17 2018
##################################################

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import satnogs


class satnogs_bpsk_decoder(gr.top_block):

    def __init__(self, antenna=satnogs.not_set_antenna, baudrate=1200, bb_gain=satnogs.not_set_rx_bb_gain, cw_offset=1500, decoded_data_file_path='/tmp/.satnogs/data/data', dev_args=satnogs.not_set_dev_args, doppler_correction_per_sec=20, enable_iq_dump=0, file_path='test.ogg', if_gain=satnogs.not_set_rx_if_gain, iq_file_path='/tmp/iq.dat', lo_offset=100e3, ppm=0, rf_gain=satnogs.not_set_rx_rf_gain, rigctl_port=4532, rx_freq=100e6, rx_sdr_device='rtlsdr', udp_IP='127.0.0.1', udp_port=16887, waterfall_file_path='/tmp/waterfall.dat'):
        gr.top_block.__init__(self, "BPSK Decoder")

        ##################################################
        # Parameters
        ##################################################
        self.antenna = antenna
        self.baudrate = baudrate
        self.bb_gain = bb_gain
        self.cw_offset = cw_offset
        self.decoded_data_file_path = decoded_data_file_path
        self.dev_args = dev_args
        self.doppler_correction_per_sec = doppler_correction_per_sec
        self.enable_iq_dump = enable_iq_dump
        self.file_path = file_path
        self.if_gain = if_gain
        self.iq_file_path = iq_file_path
        self.lo_offset = lo_offset
        self.ppm = ppm
        self.rf_gain = rf_gain
        self.rigctl_port = rigctl_port
        self.rx_freq = rx_freq
        self.rx_sdr_device = rx_sdr_device
        self.udp_IP = udp_IP
        self.udp_port = udp_port
        self.waterfall_file_path = waterfall_file_path

        ##################################################
        # Variables
        ##################################################
        self.samp_rate_rx = samp_rate_rx = satnogs.hw_rx_settings[rx_sdr_device]['samp_rate']
        self.samp_per_sym = samp_per_sym = 5
        self.nfilts = nfilts = 16
        self.xlate_filter_taps = xlate_filter_taps = firdes.low_pass(1, samp_rate_rx, 125000, 25000, firdes.WIN_HAMMING, 6.76)

        self.taps = taps = firdes.low_pass(12.0, samp_rate_rx, 100e3, 60000, firdes.WIN_HAMMING, 6.76)

        self.rrc_taps = rrc_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0/float(samp_per_sym), 0.35, 11*samp_per_sym*nfilts)
        self.filter_rate = filter_rate = 250000
        self.filt_mode = filt_mode = 0.1
        self.deviation = deviation = 5000
        self.audio_samp_rate = audio_samp_rate = 48000
        self.alpha = alpha = 0.1

        ##################################################
        # Blocks
        ##################################################
        self.satnogs_waterfall_sink_0 = satnogs.waterfall_sink(max(12000, int(3*(1+alpha)*baudrate)), 0.0, 10, 1024, waterfall_file_path, 1)
        self.satnogs_udp_msg_sink_0_0 = satnogs.udp_msg_sink(udp_IP, udp_port, 1500)
        self.satnogs_tcp_rigctl_msg_source_0 = satnogs.tcp_rigctl_msg_source("127.0.0.1", rigctl_port, False, 1000/doppler_correction_per_sec, 1500)
        self.satnogs_ogg_encoder_0 = satnogs.ogg_encoder(file_path, audio_samp_rate, 1.0)
        self.satnogs_iq_sink_0 = satnogs.iq_sink(32767, iq_file_path, False, enable_iq_dump)
        self.satnogs_frame_file_sink_0_1_0 = satnogs.frame_file_sink(decoded_data_file_path, 0)
        self.satnogs_coarse_doppler_correction_cc_0 = satnogs.coarse_doppler_correction_cc(rx_freq, samp_rate_rx)
        self.satnogs_ax25_decoder_bm_0_0 = satnogs.ax25_decoder_bm('GND', 0, True, False, 1024)
        self.satnogs_ax25_decoder_bm_0 = satnogs.ax25_decoder_bm('GND', 0, True, True, 1024)
        self.osmosdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + satnogs.handle_rx_dev_args(rx_sdr_device, dev_args) )
        self.osmosdr_source_0.set_sample_rate(samp_rate_rx)
        self.osmosdr_source_0.set_center_freq(rx_freq - lo_offset, 0)
        self.osmosdr_source_0.set_freq_corr(ppm, 0)
        self.osmosdr_source_0.set_dc_offset_mode(2, 0)
        self.osmosdr_source_0.set_iq_balance_mode(0, 0)
        self.osmosdr_source_0.set_gain_mode(False, 0)
        self.osmosdr_source_0.set_gain(satnogs.handle_rx_rf_gain(rx_sdr_device, rf_gain), 0)
        self.osmosdr_source_0.set_if_gain(satnogs.handle_rx_if_gain(rx_sdr_device, if_gain), 0)
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(rx_sdr_device, bb_gain), 0)
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(rx_sdr_device, antenna), 0)
        self.osmosdr_source_0.set_bandwidth(samp_rate_rx, 0)

        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, audio_samp_rate, (1+alpha)*baudrate, ((1+alpha)*baudrate)*filt_mode, firdes.WIN_HAMMING, 6.76))
        self.freq_xlating_fir_filter_xxx_0_0 = filter.freq_xlating_fir_filter_ccf(audio_samp_rate/(samp_per_sym*baudrate), (firdes.low_pass(1, audio_samp_rate, (1+alpha)*baudrate, (1+alpha)*baudrate*filt_mode)), cw_offset/1200.0*baudrate, audio_samp_rate)
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(int(samp_rate_rx/filter_rate), (xlate_filter_taps), lo_offset, samp_rate_rx)
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(samp_per_sym, 0.063, (rrc_taps), nfilts, nfilts/2, 1.5, 1)
        self.digital_costas_loop_cc_0_0_0_0 = digital.costas_loop_cc(0.063, 2, False)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_multiply_xx_0_0 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_complex_to_real_0_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blks2_rational_resampler_xxx_1_0 = filter.rational_resampler_ccc(
                interpolation=max(12000, int(3*(1+alpha)*baudrate)),
                decimation=48000,
                taps=None,
                fractional_bw=None,
        )
        self.blks2_rational_resampler_xxx_1 = filter.rational_resampler_ccc(
                interpolation=audio_samp_rate,
                decimation=int(samp_rate_rx/(samp_rate_rx/filter_rate)),
                taps=None,
                fractional_bw=None,
        )
        self.analog_sig_source_x_0 = analog.sig_source_c(audio_samp_rate, analog.GR_COS_WAVE, cw_offset/1200.0*baudrate, 1, 0)
        self.analog_agc2_xx_0_0 = analog.agc2_cc(0.01, 0.001, 0.015, 0.0)
        self.analog_agc2_xx_0_0.set_max_gain(65536)
        self.analog_agc2_xx_0 = analog.agc2_cc(0.01, 0.001, 0.5, 1.0)
        self.analog_agc2_xx_0.set_max_gain(65536)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.satnogs_ax25_decoder_bm_0, 'pdu'), (self.satnogs_frame_file_sink_0_1_0, 'frame'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0, 'pdu'), (self.satnogs_udp_msg_sink_0_0, 'in'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0_0, 'pdu'), (self.satnogs_frame_file_sink_0_1_0, 'frame'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0_0, 'pdu'), (self.satnogs_udp_msg_sink_0_0, 'in'))
        self.msg_connect((self.satnogs_tcp_rigctl_msg_source_0, 'freq'), (self.satnogs_coarse_doppler_correction_cc_0, 'freq'))
        self.connect((self.analog_agc2_xx_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))
        self.connect((self.analog_agc2_xx_0_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0_0, 0))
        self.connect((self.blks2_rational_resampler_xxx_1, 0), (self.blks2_rational_resampler_xxx_1_0, 0))
        self.connect((self.blks2_rational_resampler_xxx_1, 0), (self.blocks_multiply_xx_0_0, 1))
        self.connect((self.blks2_rational_resampler_xxx_1, 0), (self.low_pass_filter_0, 0))
        self.connect((self.blks2_rational_resampler_xxx_1, 0), (self.satnogs_iq_sink_0, 0))
        self.connect((self.blks2_rational_resampler_xxx_1_0, 0), (self.satnogs_waterfall_sink_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.satnogs_ogg_encoder_0, 0))
        self.connect((self.blocks_complex_to_real_0_0, 0), (self.digital_binary_slicer_fb_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.blocks_multiply_xx_0_0, 0), (self.freq_xlating_fir_filter_xxx_0_0, 0))
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.satnogs_ax25_decoder_bm_0, 0))
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.satnogs_ax25_decoder_bm_0_0, 0))
        self.connect((self.digital_costas_loop_cc_0_0_0_0, 0), (self.blocks_complex_to_real_0_0, 0))
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.digital_costas_loop_cc_0_0_0_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.blks2_rational_resampler_xxx_1, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0_0, 0), (self.analog_agc2_xx_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.analog_agc2_xx_0_0, 0))
        self.connect((self.osmosdr_source_0, 0), (self.satnogs_coarse_doppler_correction_cc_0, 0))
        self.connect((self.satnogs_coarse_doppler_correction_cc_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))

    def get_antenna(self):
        return self.antenna

    def set_antenna(self, antenna):
        self.antenna = antenna
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(self.rx_sdr_device, self.antenna), 0)

    def get_baudrate(self):
        return self.baudrate

    def set_baudrate(self, baudrate):
        self.baudrate = baudrate
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, ((1+self.alpha)*self.baudrate)*self.filt_mode, firdes.WIN_HAMMING, 6.76))
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, (1+self.alpha)*self.baudrate*self.filt_mode)))
        self.freq_xlating_fir_filter_xxx_0_0.set_center_freq(self.cw_offset/1200.0*self.baudrate)
        self.analog_sig_source_x_0.set_frequency(self.cw_offset/1200.0*self.baudrate)

    def get_bb_gain(self):
        return self.bb_gain

    def set_bb_gain(self, bb_gain):
        self.bb_gain = bb_gain
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(self.rx_sdr_device, self.bb_gain), 0)

    def get_cw_offset(self):
        return self.cw_offset

    def set_cw_offset(self, cw_offset):
        self.cw_offset = cw_offset
        self.freq_xlating_fir_filter_xxx_0_0.set_center_freq(self.cw_offset/1200.0*self.baudrate)
        self.analog_sig_source_x_0.set_frequency(self.cw_offset/1200.0*self.baudrate)

    def get_decoded_data_file_path(self):
        return self.decoded_data_file_path

    def set_decoded_data_file_path(self, decoded_data_file_path):
        self.decoded_data_file_path = decoded_data_file_path

    def get_dev_args(self):
        return self.dev_args

    def set_dev_args(self, dev_args):
        self.dev_args = dev_args

    def get_doppler_correction_per_sec(self):
        return self.doppler_correction_per_sec

    def set_doppler_correction_per_sec(self, doppler_correction_per_sec):
        self.doppler_correction_per_sec = doppler_correction_per_sec

    def get_enable_iq_dump(self):
        return self.enable_iq_dump

    def set_enable_iq_dump(self, enable_iq_dump):
        self.enable_iq_dump = enable_iq_dump

    def get_file_path(self):
        return self.file_path

    def set_file_path(self, file_path):
        self.file_path = file_path

    def get_if_gain(self):
        return self.if_gain

    def set_if_gain(self, if_gain):
        self.if_gain = if_gain
        self.osmosdr_source_0.set_if_gain(satnogs.handle_rx_if_gain(self.rx_sdr_device, self.if_gain), 0)

    def get_iq_file_path(self):
        return self.iq_file_path

    def set_iq_file_path(self, iq_file_path):
        self.iq_file_path = iq_file_path

    def get_lo_offset(self):
        return self.lo_offset

    def set_lo_offset(self, lo_offset):
        self.lo_offset = lo_offset
        self.osmosdr_source_0.set_center_freq(self.rx_freq - self.lo_offset, 0)
        self.freq_xlating_fir_filter_xxx_0.set_center_freq(self.lo_offset)

    def get_ppm(self):
        return self.ppm

    def set_ppm(self, ppm):
        self.ppm = ppm
        self.osmosdr_source_0.set_freq_corr(self.ppm, 0)

    def get_rf_gain(self):
        return self.rf_gain

    def set_rf_gain(self, rf_gain):
        self.rf_gain = rf_gain
        self.osmosdr_source_0.set_gain(satnogs.handle_rx_rf_gain(self.rx_sdr_device, self.rf_gain), 0)

    def get_rigctl_port(self):
        return self.rigctl_port

    def set_rigctl_port(self, rigctl_port):
        self.rigctl_port = rigctl_port

    def get_rx_freq(self):
        return self.rx_freq

    def set_rx_freq(self, rx_freq):
        self.rx_freq = rx_freq
        self.satnogs_coarse_doppler_correction_cc_0.set_new_freq_locked(self.rx_freq)
        self.osmosdr_source_0.set_center_freq(self.rx_freq - self.lo_offset, 0)

    def get_rx_sdr_device(self):
        return self.rx_sdr_device

    def set_rx_sdr_device(self, rx_sdr_device):
        self.rx_sdr_device = rx_sdr_device
        self.set_samp_rate_rx(satnogs.hw_rx_settings[self.rx_sdr_device]['samp_rate'])
        self.osmosdr_source_0.set_gain(satnogs.handle_rx_rf_gain(self.rx_sdr_device, self.rf_gain), 0)
        self.osmosdr_source_0.set_if_gain(satnogs.handle_rx_if_gain(self.rx_sdr_device, self.if_gain), 0)
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(self.rx_sdr_device, self.bb_gain), 0)
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(self.rx_sdr_device, self.antenna), 0)

    def get_udp_IP(self):
        return self.udp_IP

    def set_udp_IP(self, udp_IP):
        self.udp_IP = udp_IP

    def get_udp_port(self):
        return self.udp_port

    def set_udp_port(self, udp_port):
        self.udp_port = udp_port

    def get_waterfall_file_path(self):
        return self.waterfall_file_path

    def set_waterfall_file_path(self, waterfall_file_path):
        self.waterfall_file_path = waterfall_file_path

    def get_samp_rate_rx(self):
        return self.samp_rate_rx

    def set_samp_rate_rx(self, samp_rate_rx):
        self.samp_rate_rx = samp_rate_rx
        self.set_xlate_filter_taps(firdes.low_pass(1, self.samp_rate_rx, 125000, 25000, firdes.WIN_HAMMING, 6.76))
        self.osmosdr_source_0.set_sample_rate(self.samp_rate_rx)
        self.osmosdr_source_0.set_bandwidth(self.samp_rate_rx, 0)

    def get_samp_per_sym(self):
        return self.samp_per_sym

    def set_samp_per_sym(self, samp_per_sym):
        self.samp_per_sym = samp_per_sym
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.samp_per_sym), 0.35, 11*self.samp_per_sym*self.nfilts))

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.samp_per_sym), 0.35, 11*self.samp_per_sym*self.nfilts))

    def get_xlate_filter_taps(self):
        return self.xlate_filter_taps

    def set_xlate_filter_taps(self, xlate_filter_taps):
        self.xlate_filter_taps = xlate_filter_taps
        self.freq_xlating_fir_filter_xxx_0.set_taps((self.xlate_filter_taps))

    def get_taps(self):
        return self.taps

    def set_taps(self, taps):
        self.taps = taps

    def get_rrc_taps(self):
        return self.rrc_taps

    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps
        self.digital_pfb_clock_sync_xxx_0.update_taps((self.rrc_taps))

    def get_filter_rate(self):
        return self.filter_rate

    def set_filter_rate(self, filter_rate):
        self.filter_rate = filter_rate

    def get_filt_mode(self):
        return self.filt_mode

    def set_filt_mode(self, filt_mode):
        self.filt_mode = filt_mode
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, ((1+self.alpha)*self.baudrate)*self.filt_mode, firdes.WIN_HAMMING, 6.76))
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, (1+self.alpha)*self.baudrate*self.filt_mode)))

    def get_deviation(self):
        return self.deviation

    def set_deviation(self, deviation):
        self.deviation = deviation

    def get_audio_samp_rate(self):
        return self.audio_samp_rate

    def set_audio_samp_rate(self, audio_samp_rate):
        self.audio_samp_rate = audio_samp_rate
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, ((1+self.alpha)*self.baudrate)*self.filt_mode, firdes.WIN_HAMMING, 6.76))
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, (1+self.alpha)*self.baudrate*self.filt_mode)))
        self.analog_sig_source_x_0.set_sampling_freq(self.audio_samp_rate)

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, ((1+self.alpha)*self.baudrate)*self.filt_mode, firdes.WIN_HAMMING, 6.76))
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((firdes.low_pass(1, self.audio_samp_rate, (1+self.alpha)*self.baudrate, (1+self.alpha)*self.baudrate*self.filt_mode)))


def argument_parser():
    description = 'A BPSK decoder block for gr-satnogs'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--antenna", dest="antenna", type="string", default=satnogs.not_set_antenna,
        help="Set antenna [default=%default]")
    parser.add_option(
        "", "--baudrate", dest="baudrate", type="intx", default=1200,
        help="Set baudrate [default=%default]")
    parser.add_option(
        "", "--bb-gain", dest="bb_gain", type="eng_float", default=eng_notation.num_to_str(satnogs.not_set_rx_bb_gain),
        help="Set bb_gain [default=%default]")
    parser.add_option(
        "", "--cw-offset", dest="cw_offset", type="eng_float", default=eng_notation.num_to_str(1500),
        help="Set cw_offset [default=%default]")
    parser.add_option(
        "", "--decoded-data-file-path", dest="decoded_data_file_path", type="string", default='/tmp/.satnogs/data/data',
        help="Set decoded_data_file_path [default=%default]")
    parser.add_option(
        "", "--dev-args", dest="dev_args", type="string", default=satnogs.not_set_dev_args,
        help="Set dev_args [default=%default]")
    parser.add_option(
        "", "--doppler-correction-per-sec", dest="doppler_correction_per_sec", type="intx", default=20,
        help="Set doppler_correction_per_sec [default=%default]")
    parser.add_option(
        "", "--enable-iq-dump", dest="enable_iq_dump", type="intx", default=0,
        help="Set enable_iq_dump [default=%default]")
    parser.add_option(
        "", "--file-path", dest="file_path", type="string", default='test.ogg',
        help="Set file_path [default=%default]")
    parser.add_option(
        "", "--if-gain", dest="if_gain", type="eng_float", default=eng_notation.num_to_str(satnogs.not_set_rx_if_gain),
        help="Set if_gain [default=%default]")
    parser.add_option(
        "", "--iq-file-path", dest="iq_file_path", type="string", default='/tmp/iq.dat',
        help="Set iq_file_path [default=%default]")
    parser.add_option(
        "", "--lo-offset", dest="lo_offset", type="eng_float", default=eng_notation.num_to_str(100e3),
        help="Set lo_offset [default=%default]")
    parser.add_option(
        "", "--ppm", dest="ppm", type="intx", default=0,
        help="Set ppm [default=%default]")
    parser.add_option(
        "", "--rf-gain", dest="rf_gain", type="eng_float", default=eng_notation.num_to_str(satnogs.not_set_rx_rf_gain),
        help="Set rf_gain [default=%default]")
    parser.add_option(
        "", "--rigctl-port", dest="rigctl_port", type="intx", default=4532,
        help="Set rigctl_port [default=%default]")
    parser.add_option(
        "", "--rx-freq", dest="rx_freq", type="eng_float", default=eng_notation.num_to_str(100e6),
        help="Set rx_freq [default=%default]")
    parser.add_option(
        "", "--rx-sdr-device", dest="rx_sdr_device", type="string", default='rtlsdr',
        help="Set rx_sdr_device [default=%default]")
    parser.add_option(
        "", "--udp-IP", dest="udp_IP", type="string", default='127.0.0.1',
        help="Set udp_IP [default=%default]")
    parser.add_option(
        "", "--udp-port", dest="udp_port", type="intx", default=16887,
        help="Set udp_port [default=%default]")
    parser.add_option(
        "", "--waterfall-file-path", dest="waterfall_file_path", type="string", default='/tmp/waterfall.dat',
        help="Set waterfall_file_path [default=%default]")
    return parser


def main(top_block_cls=satnogs_bpsk_decoder, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    tb = top_block_cls(antenna=options.antenna, baudrate=options.baudrate, bb_gain=options.bb_gain, cw_offset=options.cw_offset, decoded_data_file_path=options.decoded_data_file_path, dev_args=options.dev_args, doppler_correction_per_sec=options.doppler_correction_per_sec, enable_iq_dump=options.enable_iq_dump, file_path=options.file_path, if_gain=options.if_gain, iq_file_path=options.iq_file_path, lo_offset=options.lo_offset, ppm=options.ppm, rf_gain=options.rf_gain, rigctl_port=options.rigctl_port, rx_freq=options.rx_freq, rx_sdr_device=options.rx_sdr_device, udp_IP=options.udp_IP, udp_port=options.udp_port, waterfall_file_path=options.waterfall_file_path)
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
