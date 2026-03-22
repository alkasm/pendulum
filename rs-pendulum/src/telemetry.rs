use std::sync::{Arc, Condvar, Mutex};

use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, ElectricCurrent, Time, Torque};

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct TelemetryFrame {
    pub step: u64,
    pub sim_time: Time,
    pub theta: Angle,
    pub theta_dot: AngularVelocity,
    pub wheel_angle: Angle,
    pub wheel_speed: AngularVelocity,
    pub commanded_torque: Torque,
    pub applied_torque: Torque,
    pub available_torque: Torque,
    pub speed_ratio: f64,
    pub phase_current: ElectricCurrent,
}

#[derive(Clone)]
pub struct TelemetryStream {
    inner: Arc<Shared>,
}

pub struct TelemetryReceiver {
    inner: Arc<Shared>,
    last_seen_seq: u64,
}

pub struct TelemetrySender {
    inner: Arc<Shared>,
}

struct Shared {
    state: Mutex<State>,
    ready: Condvar,
}

struct State {
    latest: Option<TelemetryFrame>,
    seq: u64,
    publishers: usize,
}

impl TelemetryStream {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Shared {
                state: Mutex::new(State {
                    latest: None,
                    seq: 0,
                    publishers: 0,
                }),
                ready: Condvar::new(),
            }),
        }
    }

    pub fn publisher(&self) -> TelemetrySender {
        TelemetrySender::new(self.inner.clone())
    }

    pub fn subscribe(&self) -> TelemetryReceiver {
        let last_seen_seq = self
            .inner
            .state
            .lock()
            .expect("telemetry state mutex poisoned")
            .seq
            .saturating_sub(1);

        TelemetryReceiver {
            inner: self.inner.clone(),
            last_seen_seq,
        }
    }
}

pub fn channel() -> (TelemetrySender, TelemetryReceiver) {
    let stream = TelemetryStream::new();
    (stream.publisher(), stream.subscribe())
}

impl TelemetrySender {
    fn new(inner: Arc<Shared>) -> Self {
        let mut state = inner.state.lock().expect("telemetry state mutex poisoned");
        state.publishers += 1;
        drop(state);

        Self { inner }
    }

    pub fn send(&self, frame: TelemetryFrame) {
        let mut state = self
            .inner
            .state
            .lock()
            .expect("telemetry state mutex poisoned");
        state.latest = Some(frame);
        state.seq += 1;
        self.inner.ready.notify_all();
    }

    pub fn subscribe(&self) -> TelemetryReceiver {
        TelemetryStream {
            inner: self.inner.clone(),
        }
        .subscribe()
    }
}

impl Clone for TelemetrySender {
    fn clone(&self) -> Self {
        Self::new(self.inner.clone())
    }
}

impl Drop for TelemetrySender {
    fn drop(&mut self) {
        let mut state = self
            .inner
            .state
            .lock()
            .expect("telemetry state mutex poisoned");
        state.publishers = state.publishers.saturating_sub(1);
        self.inner.ready.notify_all();
    }
}

impl TelemetryReceiver {
    pub fn drain_latest(&mut self) -> Option<TelemetryFrame> {
        let state = self
            .inner
            .state
            .lock()
            .expect("telemetry state mutex poisoned");

        if state.seq == 0 || state.seq == self.last_seen_seq {
            return None;
        }

        self.last_seen_seq = state.seq;
        state.latest
    }

    pub fn recv_latest(&mut self) -> Option<TelemetryFrame> {
        let mut state = self
            .inner
            .state
            .lock()
            .expect("telemetry state mutex poisoned");

        loop {
            if state.seq != 0 && state.seq != self.last_seen_seq {
                self.last_seen_seq = state.seq;
                return state.latest;
            }

            if state.publishers == 0 {
                return None;
            }

            state = self
                .inner
                .ready
                .wait(state)
                .expect("telemetry state mutex poisoned");
        }
    }
}

pub fn drain_latest(receiver: &mut TelemetryReceiver) -> Option<TelemetryFrame> {
    receiver.drain_latest()
}

pub fn recv_latest(receiver: &mut TelemetryReceiver) -> Option<TelemetryFrame> {
    receiver.recv_latest()
}
