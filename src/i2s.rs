use crate::{
    fpioa,
    pac::{I2S0, SYSCTL},
    prelude::*,
    sysctl::{clk_en_peri, peri_reset, sysctl},
};
use embedded_i2s;
use k210_pac::{i2s0::ccr::CLK_WORD_SIZE_A, plic::targets::threshold};

pub struct Pins {
    // pub i2s_mclk: fpioa::I2S0_MCLK,   // "I2S Master Clock
    pub i2s_sclk: fpioa::I2S0_SCLK,        // "I2S Serial Clock(BCLK)
    pub i2s_ws: fpioa::I2S0_WS,            // "I2S Word Select(LRCLK)
    pub i2s_d0: Option<fpioa::I2S0_IN_D0>, // "I2S Serial Data Input 0
    pub i2s_d1: Option<fpioa::I2S0_IN_D1>, // "I2S Serial Data Input 1
    pub i2s_d2: Option<fpioa::I2S0_IN_D2>, // "I2S Serial Data Input 2
    pub i2s_d3: Option<fpioa::I2S0_IN_D3>, // "I2S Serial Data Input 3
}

/// I2S0
/// Should we take ownership of the specific pins to ensure they are set?
pub struct I2s<I2S0> {
    i2s: I2S0,
    _pins: Pins,
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
    pub fn new(i2s: I2S0, _pins: Pins) -> Self {
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
        // Clock configuration
        /*
        i2s.ccr.modify(|_, w| {
            w.clk_gate()
                .cycles12()
                .clk_word_size()
                .cycles16()
                .align_mode()
                .standard()
                .dma_tx_en()
                .clear_bit()
                .dma_rx_en()
                .clear_bit()
                .dma_divide_16()
                .clear_bit()
                .sign_expand_en()
                .clear_bit()
        }); */

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

        Self { i2s, _pins }
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

    pub fn set_sample_rate(&self, sample_rate: u32) {
        let word_size_raw = self.i2s.ccr.read().clk_word_size().bits();
        let word_size = (word_size_raw + 2) * 8;
        let pll2_clock = 1000000u32;
        // pll2_clock = sysctl_pll_get_freq(SYSCTL_PLL2);
        let threshold: u32 = pll2_clock / (sample_rate * 2u32 * u32::from(word_size) * 2u32) - 1u32;
    }
}

impl embedded_i2s::blocking::I2s<u8> for I2s<I2S0> {
    type Error = u8;
    /// Reads enough bytes to fill `left_words` and `right_words`.
    ///
    /// It is allowed for `left_words` and `right_words` to have different lengths.
    /// The read runs for `max(left_words.len(), right_words.len())` words.
    /// Incoming words after the shorter buffer has been filled will be discarded.
    fn read<'w>(
        &mut self,
        left_words: &'w mut [u8],
        right_words: &'w mut [u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }

    /// Sends `left_words` and `right_words`.
    ///
    /// It is allowed for `left_words` and `right_words` to have different lengths.
    /// The write runs for `max(left_words.len(), right_words.len())` words.
    /// The value of words sent for the shorter channel after its buffer has been sent
    /// is implementation-defined, typically `0x00`, `0xFF`, or configurable.
    fn write<'w>(
        &mut self,
        left_words: &'w [u8],
        right_words: &'w [u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }

    /// Sends `left_words` and `right_words` getting the data from iterators.
    ///
    /// It is allowed for `left_words` and `right_words` to have different lengths.
    /// The write runs for `max(left_words.len(), right_words.len())` words.
    /// The value of words sent for the shorter channel after its buffer has been sent
    /// is implementation-defined, typically `0x00`, `0xFF`, or configurable.
    fn write_iter<LW, RW>(&mut self, left_words: LW, right_words: RW) -> Result<(), Self::Error>
    where
        LW: IntoIterator<Item = u8>,
        RW: IntoIterator<Item = u8>,
    {
        todo!()
    }
}
