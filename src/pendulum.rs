use uom::si::{
    f64::Length,
    length::meter,
};

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point2 {
    pub x: Length,
    pub y: Length,
}

impl Point2 {
    pub fn new(x: Length, y: Length) -> Self {
        Self { x, y }
    }

    pub fn origin() -> Self {
        Self::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0))
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point3 {
    pub x: Length,
    pub y: Length,
    pub z: Length,
}

impl Point3 {
    pub fn new(x: Length, y: Length, z: Length) -> Self {
        Self { x, y, z }
    }

    pub fn origin() -> Self {
        Self::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyAxis3 {
    Right,
    Left,
    Up,
    Down,
    TowardViewer,
    AwayFromViewer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuAxesInBody {
    pub x_axis: BodyAxis3,
    pub y_axis: BodyAxis3,
    pub z_axis: BodyAxis3,
}

impl ImuAxesInBody {
    pub fn new(x_axis: BodyAxis3, y_axis: BodyAxis3, z_axis: BodyAxis3) -> Self {
        Self {
            x_axis,
            y_axis,
            z_axis,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RightTriangularBody {
    pub leg_x: Length,
    pub leg_y: Length,
    pub depth: Length,
}

impl RightTriangularBody {
    pub fn new(leg_x: Length, leg_y: Length, depth: Length) -> Self {
        Self {
            leg_x,
            leg_y,
            depth,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorMount {
    pub center_from_pivot: Point3,
}

impl MotorMount {
    pub fn new(center_from_pivot: Point3) -> Self {
        Self { center_from_pivot }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuMount {
    pub translation_from_motor: Point3,
    pub axes_in_body: ImuAxesInBody,
}

impl ImuMount {
    pub fn new(translation_from_motor: Point3, axes_in_body: ImuAxesInBody) -> Self {
        Self {
            translation_from_motor,
            axes_in_body,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PendulumGeometry {
    pub body: RightTriangularBody,
    pub center_of_mass_from_pivot: Point2,
    pub motor_mount: MotorMount,
    pub imu_mount: ImuMount,
}

impl PendulumGeometry {
    pub fn new(
        body: RightTriangularBody,
        center_of_mass_from_pivot: Point2,
        motor_mount: MotorMount,
        imu_mount: ImuMount,
    ) -> Self {
        Self {
            body,
            center_of_mass_from_pivot,
            motor_mount,
            imu_mount,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendulumHardware {
    pub imu_i2c_address: u8,
    pub hall_i2c_address: u8,
}

impl PendulumHardware {
    pub fn new(imu_i2c_address: u8, hall_i2c_address: u8) -> Self {
        Self {
            imu_i2c_address,
            hall_i2c_address,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pendulum {
    pub geometry: PendulumGeometry,
    pub hardware: PendulumHardware,
}

impl Pendulum {
    pub fn new(geometry: PendulumGeometry, hardware: PendulumHardware) -> Self {
        Self { geometry, hardware }
    }
}
