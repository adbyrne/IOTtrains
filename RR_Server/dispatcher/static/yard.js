'use strict';

const DAYS = ['', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'];
const DIR_WARD = { N: 'Northward', S: 'Southward' };   // full-word direction, no "bound" suffix

// Train ID convention: "23N" for scheduled trains, "X34S" for extras (engine number).
function trainIdLabel(train, direction, isExtra) {
    return `${isExtra ? 'X' : ''}${train}${direction || ''}`;
}

// Extras are requested before an engine number exists — until one is assigned,
// display "XTRA <requested time>" using the placeholder consist record's own
// key (e.g. "XTRA3") and requested_rr_time. Falls back to trainIdLabel once
// the YM has entered an engine number.
function occupantTrainId(c) {
    if (c.extra) {
        if (c.engine) return trainIdLabel(c.engine, c.direction, true);
        return `XTRA ${c.requested_rr_time ? fmtTime(c.requested_rr_time) : c.train}`;
    }
    return trainIdLabel(c.train, c.direction, false);
}

// "Eng 101 · Cab 204" — only the fields that are actually set (any consist state may have them).
function consistDetailLine(consist) {
    if (!consist) return '';
    const parts = [];
    if (consist.engine) parts.push(`Eng ${consist.engine}`);
    if (consist.caboose) parts.push(`Cab ${consist.caboose}`);
    return parts.join(' &middot; ');
}
const NUMPAD_KEYS = ['7', '8', '9', '4', '5', '6', '1', '2', '3', 'clr', '0', 'bksp'];
const STATUS_BADGE = {
    assembling:      { text: '◐ ASSEMBLING',       cls: 'badge-assembling' },
    car_block_ready: { text: '○ CAR BLOCK READY',  cls: 'badge-carblock' },
    ready:           { text: '● READY',            cls: 'badge-ready' },
    cleared:         { text: '✓ CLEARED',          cls: 'badge-cleared' },
};
const NOT_STARTED_BADGE = { text: '○ NOT STARTED', cls: 'badge-none' };

let ws = null;
let reconnectTimer = null;
let consists = {};            // train/engine number -> consist dict
let yardTracks = [];
let yardNotifications = [];
let coeTrains = [];
let yardDepartures = [];
let annulledTrains = new Set();
let pendingExtras = {};           // engine -> {direction} for new_train notifications with no consist record yet (future Path A/C use)
let _clockSnap = null;

// ── WebSocket ────────────────────────────────────────────────────────────────

function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onopen = () => {
        document.getElementById('yard-status-dot').classList.add('online');
        if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    };

    ws.onmessage = (evt) => {
        let event;
        try { event = JSON.parse(evt.data); } catch { return; }
        switch (event.type) {
            case 'initial_state':     handleInitialState(event);   break;
            case 'clock_update':      updateClock(event.clock);    break;
            case 'consist_update':    handleConsistUpdate(event);  break;
            case 'yard_notification': handleYardNotification(event); break;
        }
    };

    ws.onclose = () => {
        document.getElementById('yard-status-dot').classList.remove('online');
        reconnectTimer = setTimeout(connect, 3000);
    };

    ws.onerror = () => { ws.close(); };
}

function handleInitialState(event) {
    consists         = event.consists || {};
    yardTracks       = event.yard_tracks || [];
    yardNotifications = event.yard_notifications || [];
    coeTrains        = event.coe_trains || [];
    yardDepartures   = event.yard_departures || [];

    for (const n of yardNotifications) {
        if (n.type === 'annulment') annulledTrains.add(n.train);
        if (n.type === 'new_train') pendingExtras[n.engine] = { direction: n.direction || 'N' };
    }

    buildTrackSelect(document.getElementById('cs-track-select'), TRACK_BOARD_EXCLUDE_FN);
    buildTrackSelect(document.getElementById('ce-track-select'),
        TRACK_BOARD_EXCLUDE_FN.concat(['interchange_east', 'interchange_west', 'co_siding']));

    updateClock(event.clock);
    renderDeparting();
    renderTrackBoard();
    renderArriving();
    renderCoeFooter();
}

function handleConsistUpdate(event) {
    const c = event.consist;
    consists[c.train] = c;
    renderDeparting();
    renderTrackBoard();
}

function handleYardNotification(event) {
    const n = event.notification;
    yardNotifications.unshift(n);
    if (n.type === 'annulment') annulledTrains.add(n.train);
    if (n.type === 'new_train') pendingExtras[n.engine] = { direction: n.direction || 'N' };
    renderDeparting();
    renderArriving();
}

// ── Clock display (mirrors dispatcher.js's interpolation approach) ───────────

function _fmt12(totalMinutes) {
    totalMinutes = ((Math.round(totalMinutes) % 1440) + 1440) % 1440;
    const h24 = Math.floor(totalMinutes / 60);
    const m = totalMinutes % 60;
    const ampm = h24 < 12 ? 'AM' : 'PM';
    let h12 = h24 % 12;
    if (h12 === 0) h12 = 12;
    return `${h12}:${String(m).padStart(2, '0')} ${ampm}`;
}

function updateClock(clock) {
    if (!clock || Object.keys(clock).length === 0) {
        _clockSnap = null;
        document.getElementById('yard-clock-time').textContent = '--:-- --';
        document.getElementById('yard-clock-day').textContent = '—';
        return;
    }
    _clockSnap = {
        rrMinutes: clock.hour * 60 + clock.minute,
        speed:     clock.speed,
        running:   clock.running,
        day:       clock.day,
        receivedAt: Date.now(),
    };
    _renderClockDisplay();
}

function _renderClockDisplay() {
    if (!_clockSnap) return;
    const { rrMinutes, speed, running, day, receivedAt } = _clockSnap;
    let current = rrMinutes;
    let currentDay = day;
    if (running) {
        const elapsedSec = (Date.now() - receivedAt) / 1000;
        const added = (elapsedSec / 60) * speed;
        const total = rrMinutes + added;
        const dayOffset = Math.floor(total / 1440);
        current = ((total % 1440) + 1440) % 1440;
        currentDay = ((day - 1 + dayOffset) % 7) + 1;
    }
    document.getElementById('yard-clock-time').textContent = _fmt12(current);
    document.getElementById('yard-clock-day').textContent = DAYS[currentDay] || '—';
}

function fmtTime(t) {
    if (!t) return '?';
    const [hStr, mStr] = t.split(':');
    const h = parseInt(hStr, 10), m = parseInt(mStr, 10);
    const ampm = h < 12 ? 'AM' : 'PM';
    const h12 = h % 12 || 12;
    return `${h12}:${String(m).padStart(2, '0')} ${ampm}`;
}

// ── Departing trains panel ────────────────────────────────────────────────────

function badgeFor(state) {
    return STATUS_BADGE[state] || NOT_STARTED_BADGE;
}

function departingCardHtml(dep, isNextUp) {
    const annulled = annulledTrains.has(dep.train);
    const consist = consists[dep.train] || dep.consist;
    const cState = consist ? consist.state : null;
    const badge = badgeFor(cState);
    const trainId = occupantTrainId(consist || dep);
    const timeStr = dep.extra
        ? (consist && consist.departure_rr_time ? `Depart ${fmtTime(consist.departure_rr_time)}` : '')
        : (dep.depart ? `Depart ${fmtTime(dep.depart)}` : '');
    const detailLine = consistDetailLine(consist);

    let actionBtn = '';
    if (!annulled && cState !== 'ready' && cState !== 'cleared') {
        const action = dep.extra ? 'build-extra' : 'build-scheduled';
        actionBtn = `<button class="btn yard-action-btn" data-action="${action}" data-train="${dep.train}">Build Consist</button>`;
    }

    return `<div class="yard-card ${isNextUp ? 'yard-card-next' : ''} ${annulled ? 'yard-card-annulled' : ''}">
        ${isNextUp ? '<div class="yard-card-flag">&#9654; NEXT UP</div>' : ''}
        <div class="yard-card-title">${trainId}${timeStr ? ' &middot; ' + timeStr : ''}</div>
        ${detailLine ? `<div class="yard-card-detail">${detailLine}</div>` : ''}
        <div class="yard-card-row">
            <span class="yard-badge ${badge.cls}">${badge.text}</span>
            ${actionBtn}
        </div>
    </div>`;
}

// Extras are computed live from `consists` + `pendingExtras` rather than the
// static `yardDepartures` snapshot taken at WS-connect time, so a newly
// authorized or newly-drafted extra appears without a page reload.
function computeExtrasForDisplay() {
    const seen = new Set();
    const result = [];
    for (const c of Object.values(consists)) {
        if (!c.extra || c.state === 'cleared') continue;
        seen.add(c.train);
        result.push({ train: c.train, direction: c.direction || 'N', service: 'Extra',
                      class: null, depart: null, extra: true, consist: c });
    }
    for (const [engine, info] of Object.entries(pendingExtras)) {
        if (seen.has(engine)) continue;
        result.push({ train: engine, direction: info.direction, service: 'Extra',
                      class: null, depart: null, extra: true, consist: null });
    }
    return result;
}

function renderDeparting() {
    const scheduled = yardDepartures.filter(d => !d.extra);
    const extras    = computeExtrasForDisplay();
    const nextUpEl = document.getElementById('next-up-card');
    const listEl   = document.getElementById('departing-list');

    nextUpEl.innerHTML = scheduled.length
        ? departingCardHtml(scheduled[0], true)
        : '<div class="yard-card muted">No scheduled departures today.</div>';

    const rest = scheduled.slice(1).concat(extras);
    listEl.innerHTML = rest.length
        ? rest.map(d => departingCardHtml(d, false)).join('')
        : '<div class="yard-card muted">No further departures.</div>';
}

// ── Track board panel ─────────────────────────────────────────────────────────
// RUN (runaround), CAB (caboose storage), and YL (yard lead) are switching/
// infrastructure tracks, not places a consist parks — excluded from the board.
const TRACK_BOARD_EXCLUDE_FN = ['runaround', 'caboose', 'yard_lead'];

function renderTrackBoard() {
    const listEl = document.getElementById('track-board-list');
    listEl.innerHTML = yardTracks.filter(t => !TRACK_BOARD_EXCLUDE_FN.includes(t.function)).map(t => {
        const occupant = Object.values(consists).find(
            c => c.track_id === t.id && c.state !== 'cleared'
        );
        let body;
        if (occupant) {
            const trainId = occupantTrainId(occupant);
            const detail = consistDetailLine(occupant);
            const badge = badgeFor(occupant.state);
            body = `<span class="tb-train">${trainId}</span>` +
                   `<span class="tb-dir">${detail}</span>` +
                   `<span class="yard-badge ${badge.cls} tb-badge">${badge.text}</span>`;
        } else {
            body = '<span class="tb-empty">—— empty</span>';
        }
        return `<div class="tb-row" data-track="${t.id}">
            <span class="tb-id" title="${t.label || t.id}">${t.id}</span>
            ${body}
        </div>`;
    }).join('');
}

// ── Arriving trains panel ─────────────────────────────────────────────────────

function renderArriving() {
    const listEl = document.getElementById('arriving-list');
    const entries = yardNotifications.filter(n => n.type === 'arrival' || n.type === 'annulment');
    listEl.innerHTML = entries.length
        ? entries.map(n => {
            if (n.type === 'annulment') {
                return `<div class="yard-arrival-entry annulled">&#10007; ${n.train} — annulled</div>`;
            }
            const eta = n.expected_rr_time ? `ETA ${fmtTime(n.expected_rr_time)}` : '';
            const trainId = trainIdLabel(n.train, n.direction, false);
            return `<div class="yard-arrival-entry">&#9658; ${trainId} — ${eta}</div>`;
        }).join('')
        : '<div class="yard-card muted">No arrivals expected.</div>';
}

// ── C&O footer ─────────────────────────────────────────────────────────────────

function renderCoeFooter() {
    const wb = coeTrains.filter(t => t.direction === 'W');
    const eb = coeTrains.filter(t => t.direction === 'E');
    const fmtList = (list) => list.length
        ? list.map(t => `No.${t.number} ${fmtTime(t.wp_arrive)}/${fmtTime(t.wp_depart)}`).join('  ·  ')
        : 'see paper timetable';
    document.getElementById('coe-wb-trains').textContent = fmtList(wb);
    document.getElementById('coe-eb-trains').textContent = fmtList(eb);
}

// ── Track select widget ───────────────────────────────────────────────────────

function buildTrackSelect(containerEl, excludeFunctions = []) {
    if (containerEl.dataset.built === '1') return;
    containerEl.dataset.built = '1';
    for (const t of yardTracks) {
        if (excludeFunctions.includes(t.function)) continue;
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'btn track-select-btn';
        btn.textContent = t.id;
        btn.title = t.label || t.id;
        btn.dataset.trackId = t.id;
        containerEl.appendChild(btn);
    }
    containerEl.addEventListener('click', e => {
        const btn = e.target.closest('.track-select-btn');
        if (!btn) return;
        containerEl.querySelectorAll('.track-select-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
    });
}

function getSelectedTrack(containerEl) {
    const active = containerEl.querySelector('.track-select-btn.active');
    return active ? active.dataset.trackId : null;
}

function setSelectedTrack(containerEl, trackId) {
    containerEl.querySelectorAll('.track-select-btn').forEach(b =>
        b.classList.toggle('active', b.dataset.trackId === trackId));
}

// ── Numpad ───────────────────────────────────────────────────────────────────

function buildNumpadGrid(gridEl) {
    gridEl.innerHTML = NUMPAD_KEYS.map(k => {
        const label = k === 'clr' ? 'CLR' : k === 'bksp' ? '⌫' : k;
        return `<button type="button" class="btn numpad-btn" data-key="${k}">${label}</button>`;
    }).join('');
}

function setupNumpad(modalEl) {
    let activeInput = null;
    const activeLabel = modalEl.querySelector('[id$="-active-field"]');

    modalEl.addEventListener('click', e => {
        const target = e.target.closest('.yard-numpad-target');
        if (target) {
            if (activeInput) activeInput.classList.remove('active-target');
            activeInput = target;
            activeInput.classList.add('active-target');
            const label = target.closest('.form-row').querySelector('label');
            if (activeLabel) activeLabel.textContent = label ? label.textContent : '';
            return;
        }
        const btn = e.target.closest('.numpad-btn');
        if (btn && activeInput) {
            const key = btn.dataset.key;
            if (key === 'clr') activeInput.value = '';
            else if (key === 'bksp') activeInput.value = activeInput.value.slice(0, -1);
            else activeInput.value += key;
        }
    });
}

function parseIntOrNull(v) {
    const n = parseInt(v, 10);
    return isNaN(n) ? null : n;
}

// ── Consist Build Modal — Scheduled ───────────────────────────────────────────

let csTrain = null;

function openScheduledModal(train) {
    csTrain = train;
    const existing = consists[train] || {};
    document.getElementById('cs-title').textContent = `Build Consist — ${trainIdLabel(train, 'N', false)}`;
    document.getElementById('cs-engine').value   = existing.engine || '';
    document.getElementById('cs-caboose').value  = existing.caboose || '';
    document.getElementById('cs-loads').value    = existing.cars_loaded != null ? existing.cars_loaded : '';
    document.getElementById('cs-empties').value  = existing.cars_empty != null ? existing.cars_empty : '';
    setSelectedTrack(document.getElementById('cs-track-select'), existing.track_id);
    document.getElementById('cs-err').textContent = '';
    document.getElementById('modal-consist-scheduled').hidden = false;
}

function closeScheduledModal() {
    document.getElementById('modal-consist-scheduled').hidden = true;
}

async function submitScheduledConsist(targetState) {
    const errEl = document.getElementById('cs-err');
    errEl.textContent = '';
    const body = {
        train: csTrain,
        state: targetState,
        direction: 'N',   // WP-departing scheduled trains are always northbound
        engine: document.getElementById('cs-engine').value.trim() || null,
        caboose: document.getElementById('cs-caboose').value.trim() || null,
        cars_loaded: parseIntOrNull(document.getElementById('cs-loads').value),
        cars_empty: parseIntOrNull(document.getElementById('cs-empties').value),
        track_id: getSelectedTrack(document.getElementById('cs-track-select')),
        extra: false,
    };
    try {
        const r = await fetch('/api/yard/consist', {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
        });
        const respBody = await r.json();
        if (!r.ok) { errEl.textContent = respBody.error || 'Error'; return; }
        closeScheduledModal();
    } catch { errEl.textContent = 'Request failed'; }
}

// ── Consist Build Modal — Extra ────────────────────────────────────────────────
// Single-stage for now: the YM builds the full consist (engine + caboose +
// cars + track) before notifying the dispatcher. The dispatcher only issues
// the running-extra TO once notified, and that TO sets the approximate
// departure time on this same record (see handleConsistUpdate / app.py).

let ceTrain = null;
let ceDirection = 'N';

function openExtraModal(train) {
    ceTrain = train;
    const existing = consists[train] || { train, extra: true, direction: 'N' };
    ceDirection = existing.direction || 'N';
    document.getElementById('ce-subtitle').textContent = occupantTrainId(existing);
    document.getElementById('ce-engine').value  = existing.engine || '';
    document.getElementById('ce-caboose').value = existing.caboose || '';
    document.getElementById('ce-loads').value   = existing.cars_loaded != null ? existing.cars_loaded : '';
    document.getElementById('ce-empties').value = existing.cars_empty != null ? existing.cars_empty : '';
    setSelectedTrack(document.getElementById('ce-track-select'), existing.track_id);
    document.getElementById('ce-err').textContent = '';
    document.getElementById('modal-consist-extra').hidden = false;
}

function closeExtraModal() {
    document.getElementById('modal-consist-extra').hidden = true;
}

async function submitExtraConsist(targetState) {
    const errEl = document.getElementById('ce-err');
    errEl.textContent = '';
    const body = {
        train: ceTrain,
        state: targetState,
        extra: true,
        direction: ceDirection,
        engine: document.getElementById('ce-engine').value.trim() || null,
        caboose: document.getElementById('ce-caboose').value.trim() || null,
        cars_loaded: parseIntOrNull(document.getElementById('ce-loads').value),
        cars_empty: parseIntOrNull(document.getElementById('ce-empties').value),
        track_id: getSelectedTrack(document.getElementById('ce-track-select')),
    };
    try {
        const r = await fetch('/api/yard/consist', {
            method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
        });
        const respBody = await r.json();
        if (!r.ok) { errEl.textContent = respBody.error || 'Error'; return; }
        closeExtraModal();
    } catch { errEl.textContent = 'Request failed'; }
}

// ── Extra Request Modal ───────────────────────────────────────────────────────

let erDirection = null;

function initExtraRequestModal() {
    document.getElementById('btn-extra-request').onclick = () => {
        erDirection = null;
        document.querySelectorAll('#er-dir-group .dir-btn').forEach(b => b.classList.remove('active'));
        document.getElementById('er-loads').value = 0;
        document.getElementById('er-empties').value = 0;
        document.getElementById('er-err').textContent = '';
        document.getElementById('modal-extra-request').hidden = false;
    };

    document.getElementById('er-dir-group').addEventListener('click', e => {
        const btn = e.target.closest('.dir-btn');
        if (!btn) return;
        erDirection = btn.dataset.dir;
        document.querySelectorAll('#er-dir-group .dir-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
    });

    document.getElementById('btn-er-submit').onclick = async () => {
        const errEl = document.getElementById('er-err');
        errEl.textContent = '';
        if (!erDirection) { errEl.textContent = 'Select a direction'; return; }
        try {
            const r = await fetch('/api/yard/extra_request', {
                method: 'POST', headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    direction: erDirection,
                    approx_loads: parseIntOrNull(document.getElementById('er-loads').value),
                    approx_empties: parseIntOrNull(document.getElementById('er-empties').value),
                }),
            });
            const body = await r.json();
            if (!r.ok) { errEl.textContent = body.error || 'Error'; return; }
            document.getElementById('modal-extra-request').hidden = true;
        } catch { errEl.textContent = 'Request failed'; }
    };
}

// ── Modal close handlers (generic) ────────────────────────────────────────────

function initModalCloseHandlers() {
    document.querySelectorAll('.modal-close[data-close]').forEach(btn => {
        btn.onclick = () => { document.getElementById(btn.dataset.close).hidden = true; };
    });
    document.querySelectorAll('.modal-overlay').forEach(overlay => {
        overlay.addEventListener('click', e => {
            if (e.target === e.currentTarget) overlay.hidden = true;
        });
    });
}

// ── Boot ─────────────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    buildNumpadGrid(document.querySelector('#cs-numpad .yard-numpad-grid'));
    buildNumpadGrid(document.querySelector('#ce-numpad .yard-numpad-grid'));
    setupNumpad(document.getElementById('modal-consist-scheduled'));
    setupNumpad(document.getElementById('modal-consist-extra'));
    initModalCloseHandlers();
    initExtraRequestModal();

    document.getElementById('btn-cs-draft').onclick  = () => submitScheduledConsist('assembling');
    document.getElementById('btn-cs-submit').onclick = () => submitScheduledConsist('ready');
    document.getElementById('btn-ce-draft').onclick  = () => submitExtraConsist('assembling');
    document.getElementById('btn-ce-submit').onclick = () => submitExtraConsist('ready');

    document.getElementById('panel-departing').addEventListener('click', e => {
        const btn = e.target.closest('.yard-action-btn');
        if (!btn) return;
        const train = btn.dataset.train;
        const action = btn.dataset.action;
        if (action === 'build-scheduled') openScheduledModal(train);
        else if (action === 'build-extra' || action === 'assign-engine') openExtraModal(train);
    });

    connect();
    setInterval(_renderClockDisplay, 1000);
});
