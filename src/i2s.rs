use crate::{
    pac::{I2S0},
    prelude::*,
    sysctl::{
        clk_en_peri, peri_reset, pll_get_freq, sysctl, PllSelect, CLOCK_FREQ_PLL2_DEFAULT, PLL2,
    },
    time::Hertz,
};

/// I2S0
/// Should we take ownership of the specific pins to ensure they are set?
pub struct I2s<I2S0> {
    i2s: I2S0,
}

pub enum WordLength {
    IgnoreWordLength,
    Resolution12Bit,
    Resolution16Bit,
    Resolution20Bit,
    Resolution24Bit,
    Resolution32Bit,
}

pub enum GatingCycles {
    NoClockGating = 0x0,
    ClockCycles12 = 0x1,
    ClockCycles16 = 0x2,
    ClockCycles20 = 0x3,
    ClockCycles24 = 0x4,
}

pub enum WordSelectCycle {
    SclkCycles16 = 0x0,
    SclkCycles24 = 0x1,
    SclkCycles32 = 0x2,
}

pub enum AlignMode {
    StandardMode = 1,
    RightJustifyingMode = 2,
    LeftJustifyingMode = 4,
}

impl I2s<I2S0> {
    pub fn new(i2s: I2S0, pll2: &mut PLL2) -> Self {
        pll2.set_frequency(Hertz(CLOCK_FREQ_PLL2_DEFAULT));

        // sysctl_clock_enable(SYSCTL_CLOCK_I2S0 + device_num);
        clk_en_peri().modify(|_, w| w.i2s0_clk_en().set_bit());
        // sysctl_reset(SYSCTL_RESET_I2S0 + device_num);
        peri_reset().modify(|_, w| w.i2s0_reset().set_bit());
        // sysctl_clock_set_threshold(SYSCTL_THRESHOLD_I2S0 + device_num, 7);
        sysctl().clk_th3.modify(|_, w| w.i2s0_clk().variant(7));

        /*96k:5,44k:12,24k:23,22k:25 16k:35 sampling*/
        /*sample rate*32bit*2 =75MHz/((N+1)*2) */
        // sysctl_clock_enable(7 + device_num);
        // Clock enable
        i2s.cer.modify(|_, w| w.clken().clear_bit());

        // i2s_set_enable(device_num, 1);
        i2s.ier.modify(|_, w| w.ien().set_bit());
        // i2s_disable_block(device_num, I2S_RECEIVER);
        i2s.irer.modify(|_, w| w.rxen().clear_bit());
        // i2s_disable_block(device_num, I2S_TRANSMITTER);
        i2s.iter.modify(|_, w| w.txen().clear_bit());

        // i2s.i2s_comp_param_1
        i2s.rxffr.modify(|_, w| w.rxffr().set_bit());
        i2s.txffr.modify(|_, w| w.rxffr().set_bit());

        // Interrupts
        // Enable if channel mask != 0x3
        i2s.channel.iter().for_each(|chan| {
            // i2s_set_mask_interrupt(device_num, I2S_CHANNEL_0 + i, 1, 1, 1, 1);
            chan.imr.write(|w| {
                w.rxdam()
                    .set_bit()
                    .rxfom()
                    .set_bit()
                    .txfem()
                    .set_bit()
                    .txfom()
                    .set_bit()
            });
            // Why is this one executed for each channel?
            //  i2s_receive_enable(device_num, I2S_CHANNEL_0 + i);
            i2s.irer.modify(|_, w| w.rxen().set_bit());
            chan.rer.write(|w| w.rxchenx().set_bit());
        });
        // Or depending on channel mask: if((channel_mask & 0x3) == 0x3)
        // i2s_recv_channel_enable(device_num, I2S_CHANNEL_0 + i, 0);
        // Which is: chan.rer.write(|w| w.rxchenx().set_bit()); for each channel.

        // i2s_set_sign_expand_en(device_num, 1);
        // and i2s_receive_dma_enable(device_num, 1);
        i2s.ccr
            .modify(|_, w| w.sign_expand_en().set_bit().dma_rx_en().set_bit());

        Self { i2s }
    }

    pub fn set_rx_word_length(&self, word_length: WordLength) {
        self.i2s.channel.iter().for_each(|chan| {
            chan.rcr.modify(|_, w| match word_length {
                WordLength::IgnoreWordLength => w.wlen().ignore(),
                WordLength::Resolution12Bit => w.wlen().resolution12(),
                WordLength::Resolution16Bit => w.wlen().resolution16(),
                WordLength::Resolution20Bit => w.wlen().resolution20(),
                WordLength::Resolution24Bit => w.wlen().resolution24(),
                WordLength::Resolution32Bit => w.wlen().resolution32(),
            })
        })
    }

    pub fn set_tx_word_length(&self, word_length: WordLength) {
        self.i2s.channel.iter().for_each(|chan| unsafe {
            chan.tcr.modify(|_, w| match word_length {
                WordLength::IgnoreWordLength => w.wlen().bits(0x00),
                WordLength::Resolution12Bit => w.wlen().bits(0x01),
                WordLength::Resolution16Bit => w.wlen().bits(0x02),
                WordLength::Resolution20Bit => w.wlen().bits(0x03),
                WordLength::Resolution24Bit => w.wlen().bits(0x04),
                WordLength::Resolution32Bit => w.wlen().bits(0x05),
            })
        })
    }

    pub fn configure_master(
        &self,
        word_select_cycle: WordSelectCycle,
        gating_cycle: GatingCycles,
        word_mode: AlignMode,
    ) {
        self.i2s.ccr.modify(|_, w| {
            let w = match word_select_cycle {
                WordSelectCycle::SclkCycles16 => w.clk_word_size().cycles16(),
                WordSelectCycle::SclkCycles24 => w.clk_word_size().cycles24(),
                WordSelectCycle::SclkCycles32 => w.clk_word_size().cycles32(),
            };
            let w = match gating_cycle {
                GatingCycles::NoClockGating => w.clk_gate().no(),
                GatingCycles::ClockCycles12 => w.clk_gate().cycles12(),
                GatingCycles::ClockCycles16 => w.clk_gate().cycles16(),
                GatingCycles::ClockCycles20 => w.clk_gate().cycles20(),
                GatingCycles::ClockCycles24 => w.clk_gate().cycles24(),
            };
            let w = match word_mode {
                AlignMode::StandardMode => w.align_mode().standard(),
                AlignMode::RightJustifyingMode => w.align_mode().right(),
                AlignMode::LeftJustifyingMode => w.align_mode().left(),
            };
            w
        });
        self.i2s.cer.modify(|_, w| w.clken().set_bit());
    }

    pub fn set_sample_rate(&self, sample_rate: Hertz) {
        let sample_rate = sample_rate.0;
        let word_size_raw = self.i2s.ccr.read().clk_word_size().bits();
        let word_size = (word_size_raw + 2) * 8;
        let pll2_clock = pll_get_freq(PllSelect::Pll2).0;
        let threshold: u32 = pll2_clock / (sample_rate * 2u32 * u32::from(word_size) * 2u32) - 1u32;
        sysctl()
            .clk_th4
            .modify(|_, w| w.i2s0_mclk().variant(threshold as u8));
    }

    pub fn receive_chan<const SIZE: usize>(
        &self,
        chan_id: usize,
        left_words: &mut [u32; SIZE],
        right_words: &mut [u32; SIZE],
    ) {
        let mut i: usize = 0;
        if chan_id >= self.i2s.channel.len() {
            return;
        }

        let chan = &self.i2s.channel[chan_id];
        // Reset the RX overrun
        chan.ror.read();
        while i < SIZE {
            if chan.isr.read().rxda().bit_is_set() {
                left_words[i] = chan.left_rxtx.read().bits().into();
                right_words[i] = chan.right_rxtx.read().bits().into();
                i = i + 1;
            }
        }
    }

    pub fn receive_all<const SIZE: usize>(
        &self,
        left_words: &mut [[u32; SIZE]; 4],
        right_words: &mut [[u32; SIZE]; 4],
    ) {
        let mut current_byte: [usize; 4] = [0, 0, 0, 0];
        // Reset the RX overrun

        for channel in self.i2s.channel.iter() {
            channel.ror.read();
        }
        while current_byte[0] < SIZE
            && current_byte[1] < SIZE
            && current_byte[2] < SIZE
            && current_byte[3] < SIZE
        {
            for chan_id in 0..self.i2s.channel.len() {
                let channel = &self.i2s.channel[chan_id];
                if current_byte[chan_id] < SIZE && channel.isr.read().rxda().bit_is_set() {
                    left_words[chan_id][current_byte[chan_id]] =
                        channel.left_rxtx.read().bits().into();
                    right_words[chan_id][current_byte[chan_id]] =
                        channel.right_rxtx.read().bits().into();
                    current_byte[chan_id] = current_byte[chan_id] + 1;
                }
            }
        }
    }
}
