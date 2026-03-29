use std::sync::{Arc, Condvar, Mutex};

pub use penproto::TelemetryPacket;

#[derive(Clone)]
pub struct PacketStream {
    inner: Arc<Shared>,
}

pub struct PacketReceiver {
    inner: Arc<Shared>,
    last_seen_seq: u64,
}

pub struct PacketSender {
    inner: Arc<Shared>,
}

struct Shared {
    state: Mutex<State>,
    ready: Condvar,
}

struct State {
    latest: Option<TelemetryPacket>,
    seq: u64,
    publishers: usize,
}

impl PacketStream {
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

    pub fn publisher(&self) -> PacketSender {
        PacketSender::new(self.inner.clone())
    }

    pub fn subscribe(&self) -> PacketReceiver {
        let last_seen_seq = self
            .inner
            .state
            .lock()
            .expect("packet state mutex poisoned")
            .seq
            .saturating_sub(1);

        PacketReceiver {
            inner: self.inner.clone(),
            last_seen_seq,
        }
    }
}

pub fn channel() -> (PacketSender, PacketReceiver) {
    let stream = PacketStream::new();
    (stream.publisher(), stream.subscribe())
}

impl PacketSender {
    fn new(inner: Arc<Shared>) -> Self {
        let mut state = inner.state.lock().expect("packet state mutex poisoned");
        state.publishers += 1;
        drop(state);

        Self { inner }
    }

    pub fn send(&self, packet: TelemetryPacket) {
        let mut state = self.inner.state.lock().expect("packet state mutex poisoned");
        state.latest = Some(packet);
        state.seq += 1;
        self.inner.ready.notify_all();
    }

    pub fn subscribe(&self) -> PacketReceiver {
        PacketStream {
            inner: self.inner.clone(),
        }
        .subscribe()
    }
}

impl Clone for PacketSender {
    fn clone(&self) -> Self {
        Self::new(self.inner.clone())
    }
}

impl Drop for PacketSender {
    fn drop(&mut self) {
        let mut state = self.inner.state.lock().expect("packet state mutex poisoned");
        state.publishers = state.publishers.saturating_sub(1);
        self.inner.ready.notify_all();
    }
}

impl PacketReceiver {
    pub fn drain_latest(&mut self) -> Option<TelemetryPacket> {
        let state = self.inner.state.lock().expect("packet state mutex poisoned");

        if state.seq == 0 || state.seq == self.last_seen_seq {
            return None;
        }

        self.last_seen_seq = state.seq;
        state.latest
    }

    pub fn recv_latest(&mut self) -> Option<TelemetryPacket> {
        let mut state = self.inner.state.lock().expect("packet state mutex poisoned");

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
                .expect("packet state mutex poisoned");
        }
    }
}

pub fn drain_latest(receiver: &mut PacketReceiver) -> Option<TelemetryPacket> {
    receiver.drain_latest()
}

pub fn recv_latest(receiver: &mut PacketReceiver) -> Option<TelemetryPacket> {
    receiver.recv_latest()
}
