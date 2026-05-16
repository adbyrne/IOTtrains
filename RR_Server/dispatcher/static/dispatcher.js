'use strict';

const DAYS = ['', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'];

let ws = null;
let reconnectTimer = null;
let toSignalStations = new Set();

// ── WebSocket ────────────────────────────────────────────────────────────────

function connect() {
    const url = `ws://${location.host}/ws`;
    ws = new WebSocket(url);

    ws.onopen = () => {
        setStatus('Connected');
        document.getElementById('status-bar').classList.remove('error');
        if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    };

    ws.onmessage = (evt) => {
        let event;
        try { event = JSON.parse(evt.data); } catch { return; }
        switch (event.type) {
            case 'initial_state':    handleInitialState(event);    break;
            case 'clock_update':     handleClockUpdate(event);     break;
            case 'station_status':   handleStationStatus(event);   break;
            case 'to_signal_update': handleToSignalUpdate(event);  break;
        }
    };

    ws.onclose = () => {
        setStatus('Disconnected — reconnecting…', true);
        reconnectTimer = setTimeout(connect, 3000);
    };

    ws.onerror = () => {
        ws.close();
    };
}

// ── Event handlers ───────────────────────────────────────────────────────────

function handleInitialState(event) {
    toSignalStations = new Set(event.to_signal_stations || []);
    buildStationGrid(event.station_ids, event.station_names);
    updateClock(event.clock);
    updateAllNextTrains(event.next_trains);
    for (const [sid, status] of Object.entries(event.stations || {})) {
        updateStationStatus(sid, status);
    }
    for (const [sid, arms] of Object.entries(event.to_signals || {})) {
        for (const [dir, sigState] of Object.entries(arms)) {
            updateToSignal(sid, dir, sigState);
        }
    }
}

function handleClockUpdate(event) {
    updateClock(event.clock);
    updateAllNextTrains(event.next_trains);
}

function handleStationStatus(event) {
    updateStationStatus(event.station_id, event.status);
}

function handleToSignalUpdate(event) {
    updateToSignal(event.station_id, event.direction, event.state);
}

// ── Clock display ────────────────────────────────────────────────────────────

function updateClock(clock) {
    if (!clock || Object.keys(clock).length === 0) {
        document.getElementById('clock-time').textContent = '--:-- --';
        document.getElementById('clock-day').textContent = '—';
        document.getElementById('clock-speed').textContent = '';
        setBadge('unknown');
        return;
    }
    document.getElementById('clock-time').textContent = clock.time || '--:-- --';
    document.getElementById('clock-day').textContent = DAYS[clock.day] || '—';
    document.getElementById('clock-speed').textContent = `${clock.speed}× speed`;
    setBadge(clock.running ? 'running' : 'paused');
}

function setBadge(state) {
    const el = document.getElementById('clock-badge');
    el.className = `clock-badge ${state}`;
    el.textContent = state === 'running' ? 'Running' : state === 'paused' ? 'Paused' : '—';
}

// ── Station table ─────────────────────────────────────────────────────────────

function buildStationGrid(stationIds, stationNames) {
    const table = document.getElementById('station-table');
    if (table.querySelectorAll('.station-row').length > 0) return; // already built
    for (const sid of stationIds) {
        const hasTo = toSignalStations.has(sid);
        const row = document.createElement('div');
        row.className = 'station-row';
        row.id = `row-${sid}`;
        row.innerHTML = `
            <span class="status-dot" id="dot-${sid}"></span>
            <span class="col-id">${sid}</span>
            <span class="col-name">${stationNames[sid] || sid}</span>
            <span class="col-to" id="to-${sid}-N">${hasTo ? toHtml(null) : ''}</span>
            <span class="col-train" id="nt-${sid}-N"></span>
            <span class="col-to" id="to-${sid}-S">${hasTo ? toHtml(null) : ''}</span>
            <span class="col-train" id="nt-${sid}-S"></span>`;
        table.appendChild(row);
    }
}

function toHtml(state) {
    if (state === 'raised')  return '<span class="to-raised"  title="Raised — stop for orders">&#x25B2;</span>';
    if (state === 'lowered') return '<span class="to-lowered" title="Lowered — clear">&#x25BC;</span>';
    return '<span class="to-none">&#x25BC;</span>';
}

function updateToSignal(sid, dir, state) {
    const el = document.getElementById(`to-${sid}-${dir}`);
    if (!el) return;
    el.innerHTML = toHtml(state);
}

function updateStationStatus(sid, status) {
    const dot = document.getElementById(`dot-${sid}`);
    if (!dot) return;
    if (status && status.online) {
        dot.classList.add('online');
        dot.title = `Online — firmware ${status.firmware || '?'}, RSSI ${status.rssi || '?'} dBm`;
    } else {
        dot.classList.remove('online');
        dot.title = 'Offline';
    }
}

function updateAllNextTrains(nextTrains) {
    if (!nextTrains) return;
    for (const [sid, dirs] of Object.entries(nextTrains)) {
        updateNextTrain(sid, 'N', dirs.N);
        updateNextTrain(sid, 'S', dirs.S);
    }
}

function updateNextTrain(sid, dir, train) {
    const el = document.getElementById(`nt-${sid}-${dir}`);
    if (!el) return;
    if (train) {
        el.innerHTML = `<span class="train-num">No.&nbsp;${train.number}</span><span class="train-time">${train.time || ''}</span>`;
    } else {
        el.innerHTML = `<span class="train-none">—</span>`;
    }
}

// ── Clock controls ───────────────────────────────────────────────────────────

async function clockCmd(action, extra = {}) {
    try {
        await fetch('/api/clock/control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action, ...extra }),
        });
    } catch (e) {
        setStatus('Command failed', true);
    }
}

function initControls() {
    document.getElementById('btn-start').onclick  = () => clockCmd('start');
    document.getElementById('btn-pause').onclick  = () => clockCmd('pause');
    document.getElementById('btn-reset').onclick  = () => {
        if (confirm('Reset clock to 00:00 Monday?')) clockCmd('reset');
    };

    for (const btn of document.querySelectorAll('.speed-btn')) {
        btn.onclick = () => {
            clockCmd('speed', { speed: parseInt(btn.dataset.speed) });
            document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
        };
    }

    document.getElementById('btn-set').onclick = () => {
        const h = parseInt(document.getElementById('set-hour').value);
        const m = parseInt(document.getElementById('set-min').value);
        const d = parseInt(document.getElementById('set-day').value);
        if (isNaN(h) || isNaN(m) || isNaN(d)) return;
        clockCmd('set', { hour: h, minute: m, day: d });
    };
}

// ── Utility ──────────────────────────────────────────────────────────────────

function setStatus(msg, isError = false) {
    const el = document.getElementById('status-bar');
    el.textContent = msg;
    el.className = isError ? 'error' : '';
}

// ── Boot ─────────────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    initControls();
    connect();
});
