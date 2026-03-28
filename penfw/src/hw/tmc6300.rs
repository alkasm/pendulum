use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    peripherals::{GPIO5, GPIO16, GPIO17, GPIO18, GPIO19, GPIO23, GPIO33, GPIO34},
};

#[derive(Clone, Copy)]
pub struct CommutationStep {
    pub name: &'static str,
    pub uh: bool,
    pub ul: bool,
    pub vh: bool,
    pub vl: bool,
    pub wh: bool,
    pub wl: bool,
}

pub const SIX_STEP_COMMUTATION: [CommutationStep; 6] = [
    CommutationStep {
        name: "U+ V-",
        uh: true,
        ul: false,
        vh: false,
        vl: true,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "U+ W-",
        uh: true,
        ul: false,
        vh: false,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ W-",
        uh: false,
        ul: false,
        vh: true,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ U-",
        uh: false,
        ul: true,
        vh: true,
        vl: false,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "W+ U-",
        uh: false,
        ul: true,
        vh: false,
        vl: false,
        wh: true,
        wl: false,
    },
    CommutationStep {
        name: "W+ V-",
        uh: false,
        ul: false,
        vh: false,
        vl: true,
        wh: true,
        wl: false,
    },
];

pub struct Tmc6300<'d> {
    enable: Output<'d>,
    diag: Input<'d>,
    uh: Output<'d>,
    ul: Output<'d>,
    vh: Output<'d>,
    vl: Output<'d>,
    wh: Output<'d>,
    wl: Output<'d>,
}

impl<'d> Tmc6300<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        enable: GPIO5<'d>,
        diag: GPIO34<'d>,
        uh: GPIO16<'d>,
        ul: GPIO17<'d>,
        vh: GPIO18<'d>,
        vl: GPIO23<'d>,
        wh: GPIO19<'d>,
        wl: GPIO33<'d>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            diag: Input::new(diag, InputConfig::default()),
            uh: Output::new(uh, Level::Low, OutputConfig::default()),
            ul: Output::new(ul, Level::Low, OutputConfig::default()),
            vh: Output::new(vh, Level::Low, OutputConfig::default()),
            vl: Output::new(vl, Level::Low, OutputConfig::default()),
            wh: Output::new(wh, Level::Low, OutputConfig::default()),
            wl: Output::new(wl, Level::Low, OutputConfig::default()),
        }
    }

    pub fn enable(&mut self) {
        self.enable.set_high();
    }

    pub fn disable(&mut self) {
        self.enable.set_low();
    }

    pub fn diag_is_high(&self) -> bool {
        self.diag.is_high()
    }

    pub fn apply_step(&mut self, step: CommutationStep) {
        self.uh
            .set_level(if step.uh { Level::High } else { Level::Low });
        self.ul
            .set_level(if step.ul { Level::High } else { Level::Low });
        self.vh
            .set_level(if step.vh { Level::High } else { Level::Low });
        self.vl
            .set_level(if step.vl { Level::High } else { Level::Low });
        self.wh
            .set_level(if step.wh { Level::High } else { Level::Low });
        self.wl
            .set_level(if step.wl { Level::High } else { Level::Low });
    }
}
