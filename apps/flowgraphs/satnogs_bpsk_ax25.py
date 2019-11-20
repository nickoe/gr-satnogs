#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: satnogs_bpsk_ax25
# Author: Manolis Surligas (surligas@gmail.com), Patrick Dohmen (DL4PD)
# Description: BPSK AX.25 decoder
# Generated: Wed Nov 20 17:41:41 2019
##################################################


from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.filter import pfb
from optparse import OptionParser
import math
import osmosdr
import satnogs
import time


class satnogs_bpsk_ax25(gr.top_block):

    def __init__(self, antenna=satnogs.not_set_antenna, baudrate=9600.0, bb_gain=satnogs.not_set_rx_bb_gain, decoded_data_file_path='/tmp/.satnogs/data/data', dev_args=satnogs.not_set_dev_args, doppler_correction_per_sec=1000, enable_iq_dump=0, excess_bw=0.35, file_path='test.wav', if_gain=satnogs.not_set_rx_if_gain, iq_file_path='/tmp/iq.dat', lo_offset=100e3, max_cfo=1000.0, ppm=0, rf_gain=satnogs.not_set_rx_rf_gain, rigctl_port=4532, rx_freq=100e6, rx_sdr_device='usrpb200', samp_rate_rx=satnogs.not_set_samp_rate_rx, udp_IP='127.0.0.1', udp_port=16887, waterfall_file_path='/tmp/waterfall.dat'):
        gr.top_block.__init__(self, "satnogs_bpsk_ax25")

        ##################################################
        # Parameters
        ##################################################
        self.antenna = antenna
        self.baudrate = baudrate
        self.bb_gain = bb_gain
        self.decoded_data_file_path = decoded_data_file_path
        self.dev_args = dev_args
        self.doppler_correction_per_sec = doppler_correction_per_sec
        self.enable_iq_dump = enable_iq_dump
        self.excess_bw = excess_bw
        self.file_path = file_path
        self.if_gain = if_gain
        self.iq_file_path = iq_file_path
        self.lo_offset = lo_offset
        self.max_cfo = max_cfo
        self.ppm = ppm
        self.rf_gain = rf_gain
        self.rigctl_port = rigctl_port
        self.rx_freq = rx_freq
        self.rx_sdr_device = rx_sdr_device
        self.samp_rate_rx = samp_rate_rx
        self.udp_IP = udp_IP
        self.udp_port = udp_port
        self.waterfall_file_path = waterfall_file_path

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 4
        self.nfilts = nfilts = 32
        self.rrc_taps = rrc_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0/float(sps), excess_bw, 11*sps*nfilts)

        self.bpsk_constellation = bpsk_constellation = digital.constellation_bpsk().base()

        self.audio_samp_rate = audio_samp_rate = 48000

        ##################################################
        # Blocks
        ##################################################
        self.satnogs_waterfall_sink_0 = satnogs.waterfall_sink(audio_samp_rate, 0.0, 10, 1024, waterfall_file_path, 1)
        self.satnogs_udp_msg_sink_0_0 = satnogs.udp_msg_sink(udp_IP, udp_port, 1500)
        self.satnogs_tcp_rigctl_msg_source_0 = satnogs.tcp_rigctl_msg_source("172.28.1.2", rigctl_port, False, 1000, 1500)
        self.satnogs_ogg_encoder_0 = satnogs.ogg_encoder(file_path, audio_samp_rate, 1.0)
        self.satnogs_iq_sink_0 = satnogs.iq_sink(16768, iq_file_path, False, enable_iq_dump)
        self.satnogs_frame_file_sink_0_1_0 = satnogs.frame_file_sink(decoded_data_file_path, 0)
        self.satnogs_coarse_doppler_correction_cc_0 = satnogs.coarse_doppler_correction_cc(rx_freq, satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx))
        self.satnogs_ax25_decoder_bm_0_0 = satnogs.ax25_decoder_bm('GND', 0, True, False, 1024)
        self.satnogs_ax25_decoder_bm_0 = satnogs.ax25_decoder_bm('GND', 0, True, True, 1024)
        self.pfb_arb_resampler_xxx_0_0 = pfb.arb_resampler_ccf(
        	  (1.0 * sps*baudrate)/satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx),
                  taps=None,
        	  flt_size=32)
        self.pfb_arb_resampler_xxx_0_0.declare_sample_delay(0)

        self.pfb_arb_resampler_xxx_0 = pfb.arb_resampler_ccf(
        	  audio_samp_rate/satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx),
                  taps=None,
        	  flt_size=32)
        self.pfb_arb_resampler_xxx_0.declare_sample_delay(0)

        self.osmosdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + satnogs.handle_rx_dev_args(rx_sdr_device, dev_args) )
        self.osmosdr_source_0.set_sample_rate(satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx))
        self.osmosdr_source_0.set_center_freq(rx_freq - lo_offset, 0)
        self.osmosdr_source_0.set_freq_corr(ppm, 0)
        self.osmosdr_source_0.set_dc_offset_mode(2, 0)
        self.osmosdr_source_0.set_iq_balance_mode(0, 0)
        self.osmosdr_source_0.set_gain_mode(False, 0)
        self.osmosdr_source_0.set_gain(satnogs.handle_rx_rf_gain(rx_sdr_device, rf_gain), 0)
        self.osmosdr_source_0.set_if_gain(satnogs.handle_rx_if_gain(rx_sdr_device, if_gain), 0)
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(rx_sdr_device, bb_gain), 0)
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(rx_sdr_device, antenna), 0)
        self.osmosdr_source_0.set_bandwidth(satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx), 0)

        self.low_pass_filter_0_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, audio_samp_rate, ((1.0 + excess_bw) * baudrate/2.0) + min(baudrate, abs(max_cfo)), baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, sps*baudrate, ((1.0 + excess_bw) * baudrate/2.0) + min(baudrate, abs(max_cfo)), baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps, 2.0 * math.pi/100.0, (rrc_taps), nfilts, nfilts/2, 1.5, 1)
        self.digital_costas_loop_cc_0_0 = digital.costas_loop_cc(2.0 * math.pi / 100.0, 2, True)
        self.digital_constellation_receiver_cb_0 = digital.constellation_receiver_cb(bpsk_constellation, 2.0 * math.pi/100.0, -0.25, 0.25)
        self.blocks_rotator_cc_0_0 = blocks.rotator_cc(2.0 * math.pi * (1200.0 / audio_samp_rate))
        self.blocks_rotator_cc_0 = blocks.rotator_cc(-2.0 * math.pi * (lo_offset / satnogs.handle_samp_rate_rx(rx_sdr_device, samp_rate_rx)))
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.analog_agc2_xx_0_0 = analog.agc2_cc(0.01, 0.001, 0.015, 1.0)
        self.analog_agc2_xx_0_0.set_max_gain(65536)
        self.analog_agc2_xx_0 = analog.agc2_cc(1e-3, 1e-3, 0.5, 1.0)
        self.analog_agc2_xx_0.set_max_gain(65536)



        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.satnogs_ax25_decoder_bm_0, 'pdu'), (self.satnogs_frame_file_sink_0_1_0, 'frame'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0, 'pdu'), (self.satnogs_udp_msg_sink_0_0, 'in'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0_0, 'pdu'), (self.satnogs_frame_file_sink_0_1_0, 'frame'))
        self.msg_connect((self.satnogs_ax25_decoder_bm_0_0, 'pdu'), (self.satnogs_udp_msg_sink_0_0, 'in'))
        self.msg_connect((self.satnogs_tcp_rigctl_msg_source_0, 'freq'), (self.satnogs_coarse_doppler_correction_cc_0, 'freq'))
        self.connect((self.analog_agc2_xx_0, 0), (self.pfb_arb_resampler_xxx_0_0, 0))
        self.connect((self.analog_agc2_xx_0_0, 0), (self.low_pass_filter_0_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.satnogs_ogg_encoder_0, 0))
        self.connect((self.blocks_rotator_cc_0, 0), (self.satnogs_coarse_doppler_correction_cc_0, 0))
        self.connect((self.blocks_rotator_cc_0_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.digital_constellation_receiver_cb_0, 0), (self.satnogs_ax25_decoder_bm_0, 0))
        self.connect((self.digital_constellation_receiver_cb_0, 0), (self.satnogs_ax25_decoder_bm_0_0, 0))
        self.connect((self.digital_costas_loop_cc_0_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.digital_constellation_receiver_cb_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.digital_costas_loop_cc_0_0, 0))
        self.connect((self.low_pass_filter_0_0, 0), (self.blocks_rotator_cc_0_0, 0))
        self.connect((self.osmosdr_source_0, 0), (self.blocks_rotator_cc_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0, 0), (self.analog_agc2_xx_0_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0, 0), (self.satnogs_iq_sink_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0, 0), (self.satnogs_waterfall_sink_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.satnogs_coarse_doppler_correction_cc_0, 0), (self.analog_agc2_xx_0, 0))
        self.connect((self.satnogs_coarse_doppler_correction_cc_0, 0), (self.pfb_arb_resampler_xxx_0, 0))

    def get_antenna(self):
        return self.antenna

    def set_antenna(self, antenna):
        self.antenna = antenna
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(self.rx_sdr_device, self.antenna), 0)

    def get_baudrate(self):
        return self.baudrate

    def set_baudrate(self, baudrate):
        self.baudrate = baudrate
        self.pfb_arb_resampler_xxx_0_0.set_rate((1.0 * self.sps*self.baudrate)/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.sps*self.baudrate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))

    def get_bb_gain(self):
        return self.bb_gain

    def set_bb_gain(self, bb_gain):
        self.bb_gain = bb_gain
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(self.rx_sdr_device, self.bb_gain), 0)

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

    def get_excess_bw(self):
        return self.excess_bw

    def set_excess_bw(self, excess_bw):
        self.excess_bw = excess_bw
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.sps), self.excess_bw, 11*self.sps*self.nfilts))
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.sps*self.baudrate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))

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
        self.blocks_rotator_cc_0.set_phase_inc(-2.0 * math.pi * (self.lo_offset / satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx)))

    def get_max_cfo(self):
        return self.max_cfo

    def set_max_cfo(self, max_cfo):
        self.max_cfo = max_cfo
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.sps*self.baudrate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))

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
        self.pfb_arb_resampler_xxx_0_0.set_rate((1.0 * self.sps*self.baudrate)/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.pfb_arb_resampler_xxx_0.set_rate(self.audio_samp_rate/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.osmosdr_source_0.set_sample_rate(satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.osmosdr_source_0.set_gain(satnogs.handle_rx_rf_gain(self.rx_sdr_device, self.rf_gain), 0)
        self.osmosdr_source_0.set_if_gain(satnogs.handle_rx_if_gain(self.rx_sdr_device, self.if_gain), 0)
        self.osmosdr_source_0.set_bb_gain(satnogs.handle_rx_bb_gain(self.rx_sdr_device, self.bb_gain), 0)
        self.osmosdr_source_0.set_antenna(satnogs.handle_rx_antenna(self.rx_sdr_device, self.antenna), 0)
        self.osmosdr_source_0.set_bandwidth(satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx), 0)
        self.blocks_rotator_cc_0.set_phase_inc(-2.0 * math.pi * (self.lo_offset / satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx)))

    def get_samp_rate_rx(self):
        return self.samp_rate_rx

    def set_samp_rate_rx(self, samp_rate_rx):
        self.samp_rate_rx = samp_rate_rx
        self.pfb_arb_resampler_xxx_0_0.set_rate((1.0 * self.sps*self.baudrate)/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.pfb_arb_resampler_xxx_0.set_rate(self.audio_samp_rate/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.osmosdr_source_0.set_sample_rate(satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.osmosdr_source_0.set_bandwidth(satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx), 0)
        self.blocks_rotator_cc_0.set_phase_inc(-2.0 * math.pi * (self.lo_offset / satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx)))

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

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.sps), self.excess_bw, 11*self.sps*self.nfilts))
        self.pfb_arb_resampler_xxx_0_0.set_rate((1.0 * self.sps*self.baudrate)/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.sps*self.baudrate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.sps), self.excess_bw, 11*self.sps*self.nfilts))

    def get_rrc_taps(self):
        return self.rrc_taps

    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps
        self.digital_pfb_clock_sync_xxx_0.update_taps((self.rrc_taps))

    def get_bpsk_constellation(self):
        return self.bpsk_constellation

    def set_bpsk_constellation(self, bpsk_constellation):
        self.bpsk_constellation = bpsk_constellation

    def get_audio_samp_rate(self):
        return self.audio_samp_rate

    def set_audio_samp_rate(self, audio_samp_rate):
        self.audio_samp_rate = audio_samp_rate
        self.pfb_arb_resampler_xxx_0.set_rate(self.audio_samp_rate/satnogs.handle_samp_rate_rx(self.rx_sdr_device, self.samp_rate_rx))
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.audio_samp_rate, ((1.0 + self.excess_bw) * self.baudrate/2.0) + min(self.baudrate, abs(self.max_cfo)), self.baudrate / 10.0, firdes.WIN_HAMMING, 6.76))
        self.blocks_rotator_cc_0_0.set_phase_inc(2.0 * math.pi * (1200.0 / self.audio_samp_rate))


def argument_parser():
    description = 'BPSK AX.25 decoder'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--antenna", dest="antenna", type="string", default=satnogs.not_set_antenna,
        help="Set antenna [default=%default]")
    parser.add_option(
        "", "--baudrate", dest="baudrate", type="eng_float", default=eng_notation.num_to_str(9600.0),
        help="Set baudrate [default=%default]")
    parser.add_option(
        "", "--bb-gain", dest="bb_gain", type="eng_float", default=eng_notation.num_to_str(satnogs.not_set_rx_bb_gain),
        help="Set bb_gain [default=%default]")
    parser.add_option(
        "", "--decoded-data-file-path", dest="decoded_data_file_path", type="string", default='/tmp/.satnogs/data/data',
        help="Set decoded_data_file_path [default=%default]")
    parser.add_option(
        "", "--dev-args", dest="dev_args", type="string", default=satnogs.not_set_dev_args,
        help="Set dev_args [default=%default]")
    parser.add_option(
        "", "--doppler-correction-per-sec", dest="doppler_correction_per_sec", type="intx", default=1000,
        help="Set doppler_correction_per_sec [default=%default]")
    parser.add_option(
        "", "--enable-iq-dump", dest="enable_iq_dump", type="intx", default=0,
        help="Set enable_iq_dump [default=%default]")
    parser.add_option(
        "", "--excess-bw", dest="excess_bw", type="eng_float", default=eng_notation.num_to_str(0.35),
        help="Set excess_bw [default=%default]")
    parser.add_option(
        "", "--file-path", dest="file_path", type="string", default='test.wav',
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
        "", "--max-cfo", dest="max_cfo", type="eng_float", default=eng_notation.num_to_str(1000.0),
        help="Set max_cfo [default=%default]")
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
        "", "--rx-sdr-device", dest="rx_sdr_device", type="string", default='usrpb200',
        help="Set rx_sdr_device [default=%default]")
    parser.add_option(
        "", "--samp-rate-rx", dest="samp_rate_rx", type="eng_float", default=eng_notation.num_to_str(satnogs.not_set_samp_rate_rx),
        help="Set samp_rate_rx [default=%default]")
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


def main(top_block_cls=satnogs_bpsk_ax25, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    tb = top_block_cls(antenna=options.antenna, baudrate=options.baudrate, bb_gain=options.bb_gain, decoded_data_file_path=options.decoded_data_file_path, dev_args=options.dev_args, doppler_correction_per_sec=options.doppler_correction_per_sec, enable_iq_dump=options.enable_iq_dump, excess_bw=options.excess_bw, file_path=options.file_path, if_gain=options.if_gain, iq_file_path=options.iq_file_path, lo_offset=options.lo_offset, max_cfo=options.max_cfo, ppm=options.ppm, rf_gain=options.rf_gain, rigctl_port=options.rigctl_port, rx_freq=options.rx_freq, rx_sdr_device=options.rx_sdr_device, samp_rate_rx=options.samp_rate_rx, udp_IP=options.udp_IP, udp_port=options.udp_port, waterfall_file_path=options.waterfall_file_path)
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
