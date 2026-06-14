'use strict';

const DAYS = ['', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'];
const DIR_WORD = { N: 'North', S: 'South' };

const ALL_FORMS = [
    { letter: 'A', desc: 'Fixing Meeting Points for Opposing Trains' },
    { letter: 'B', desc: 'Authorizing a Train to Pass or Run Ahead' },
    { letter: 'C', desc: 'Giving Right Over Another Train' },
    { letter: 'E', desc: 'Time Orders' },
    { letter: 'F', desc: 'Creating Sections' },
    { letter: 'G', desc: 'Authorizing Extra Trains' },
    { letter: 'H', desc: 'Work Extras' },
    { letter: 'J', desc: 'Holding Order' },
    { letter: 'K', desc: 'Annulling a Schedule or a Section' },
    { letter: 'L', desc: 'Annulling an Order' },
    { letter: 'M', desc: 'Annulling Part of an Order' },
    { letter: 'P', desc: 'Superseding an Order or Part of an Order' },
    { letter: 'R', desc: 'Providing for a Movement Against the Current of Traffic' },
    { letter: 'S', desc: 'Providing for Use of a Section of Double or More Tracks as Single Track' },
    { letter: 'T', desc: 'Notice of New Timetable' },
    { letter: 'U', desc: 'Advance Authority to Proceed from ABS Stop Signal' },
    { letter: 'V', desc: 'Check of Trains' },
    { letter: 'W', desc: 'Change in Clearance or Register Requirements' },
    { letter: 'X', desc: 'Slow Track Conditions' },
    { letter: 'Y', desc: 'Maintenance of Way Conditional Stop' },
    { letter: 'Z', desc: 'Relief of Flag Protection' },
];

let ws = null;
let reconnectTimer = null;
let toSignalStations = new Set();
let stationIds = [];
let stationNames = {};
let toTypes = {};       // to_types.json content from server
let activeForms = [];   // active form letters from server
let layoutRules = {};   // layout operating rules from server
let toLogData = [];     // local mirror of issued TOs (newest first)
let _signalStates = {};  // { sid: { N: 'raised'|'lowered', S: 'raised'|'lowered' } }
let stationData = {};   // { sid: { types, flagging_required, siding_length_cars } }
let extraTimes = {};    // { sid: { N: minutes|null, S: minutes|null } }

// Clock interpolation state — updated on each clock_update event
let _clockSnap = null;  // { rrMinutes, speed, running, day, receivedAt }

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
            case 'os_report':        handleOsReport(event);        break;
            case 'to_issued':        handleToIssued(event);        break;
            case 'to_ack':           handleToAck(event);           break;
            case 'to_train_rcvd':   handleToTrainRcvd(event);     break;
        }
    };

    ws.onclose = () => {
        setStatus('Disconnected — reconnecting…', true);
        reconnectTimer = setTimeout(connect, 3000);
    };

    ws.onerror = () => { ws.close(); };
}

// ── Event handlers ───────────────────────────────────────────────────────────

function handleInitialState(event) {
    stationIds  = event.station_ids  || [];
    stationNames = event.station_names || {};
    toSignalStations = new Set(event.to_signal_stations || []);
    toTypes     = event.to_types     || {};
    activeForms = event.active_forms  || [];
    layoutRules = event.layout_rules  || {};
    toLogData   = event.to_log       || [];
    stationData = event.station_data  || {};
    extraTimes  = event.extra_times   || {};

    buildStationGrid(stationIds, stationNames);
    updateClock(event.clock);
    updateAllNextTrains(event.next_trains);
    for (const [sid, status] of Object.entries(event.stations || {}))
        updateStationStatus(sid, status);
    for (const [sid, arms] of Object.entries(event.to_signals || {}))
        for (const [dir, sigState] of Object.entries(arms))
            updateToSignal(sid, dir, sigState);
    rebuildOsLog(event.os_log || []);
    refreshToLog();
    populateToTypeSelect();
    populateToStationChecks();
    populateFormSelect();
    updateAllSignalArms();
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

function handleToIssued(event) {
    toLogData.unshift(event.to);
    refreshToLog();
    updateAllSignalArms();
}

function handleToAck(event) {
    const entry = toLogData.find(e => e.seq === event.seq);
    if (entry) entry.acks[event.station_id] = event.ack;
    refreshToLog();
    updateAllSignalArms();
}

function handleToTrainRcvd(event) {
    const entry = toLogData.find(e => e.seq === event.seq);
    if (entry) {
        if (!entry.train_acks) entry.train_acks = {};
        entry.train_acks[event.train] = event.rr_time;
    }
    refreshToLog();
}

// ── Clock display ────────────────────────────────────────────────────────────

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
        document.getElementById('clock-time').textContent = '--:-- --';
        document.getElementById('clock-day').textContent = '—';
        document.getElementById('clock-speed').textContent = '';
        setBadge('unknown');
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
    document.getElementById('clock-time').textContent = _fmt12(current);
    document.getElementById('clock-day').textContent = DAYS[currentDay] || '—';
    document.getElementById('clock-speed').textContent = `${speed}× speed`;
    setBadge(running ? 'running' : 'paused');
}

function setBadge(state) {
    const el = document.getElementById('clock-badge');
    el.className = `clock-badge ${state}`;
    el.textContent = state === 'running' ? 'Running' : state === 'paused' ? 'Paused' : '—';
}

// ── Station table ─────────────────────────────────────────────────────────────

function buildStationGrid(ids, names) {
    const table = document.getElementById('station-table');
    if (table.querySelectorAll('.station-row').length > 0) return;
    for (const sid of ids) {
        const hasTo = toSignalStations.has(sid);
        const sd = stationData[sid] || {};
        const et = extraTimes[sid]  || {};

        // Station sub-text: type codes, flagging mark, siding capacity
        const typesStr = (sd.types || []).join(' · ');
        const flagMark = sd.flagging_required ? ' <span class="col-flag" title="Flagging required">⚑</span>' : '';
        const sidingStr = sd.siding_length_cars != null
            ? `<span class="col-siding">${sd.siding_length_cars}&nbsp;cars</span>` : '';
        const metaParts = [typesStr, sidingStr].filter(Boolean).join(' · ');
        const metaLine = metaParts ? `<br><span class="col-meta">${metaParts}</span>` : '';

        // Extra train running times
        const nTime = et.N != null ? `<span class="rt-n">↑&nbsp;${et.N}m</span>` : '';
        const sTime = et.S != null ? `<span class="rt-s">↓&nbsp;${et.S}m</span>` : '';
        const rtHtml = (nTime || sTime)
            ? `<span class="col-runtime">${nTime}${sTime}</span>`
            : `<span class="col-runtime col-runtime-none">—</span>`;

        const row = document.createElement('div');
        row.className = 'station-row';
        row.id = `row-${sid}`;
        row.innerHTML = `
            <span class="status-dot" id="dot-${sid}"></span>
            <span class="col-id">${sid}</span>
            <span class="col-name">${names[sid] || sid}${flagMark}${metaLine}</span>
            <span class="col-to" id="to-${sid}-N">${hasTo ? armHtml(sid, 'N', 'raised') : ''}</span>
            <span class="col-train" id="nt-${sid}-N"></span>
            ${rtHtml}
            <span class="col-to" id="to-${sid}-S">${hasTo ? armHtml(sid, 'S', 'raised') : ''}</span>
            <span class="col-train" id="nt-${sid}-S"></span>`;
        table.appendChild(row);
    }
    // Bind arm button clicks (event delegation on the table)
    table.addEventListener('click', e => {
        const btn = e.target.closest('.arm-btn');
        if (!btn || btn.disabled) return;
        const sid  = btn.dataset.sid;
        const dir  = btn.dataset.dir;
        const cur  = btn.dataset.state;
        const next = cur === 'raised' ? 'lowered' : 'raised';

        // Only check block/warn when trying to RAISE (clear) a lowered signal
        if (cur === 'lowered' && !isSignalReadyToRaise(sid)) {
            const mode = layoutRules.signal_reset_mode || 'hard';
            if (mode === 'hard') return;   // button should be disabled; belt-and-suspenders
            // Soft: confirmation dialog listing outstanding TOs
            const pending = outstandingTosForStation(sid);
            const lines = pending.map(to => {
                const label = (toTypes.to_types?.[to.to_type]?.label) || to.to_type;
                return `  #${to.seq} ${label} (${to.issued_rr_time})`;
            }).join('\n');
            if (!confirm(`Unacknowledged orders remain for ${sid}:\n${lines}\n\nClear signal anyway?`)) return;
        }

        signalArmCmd(sid, dir, next);
    });
}

// C&O-type TO signals: UP arm (raised) = clear; DOWN arm (lowered) = stop for orders
function armHtml(sid, dir, state) {
    const isStopped = state === 'lowered';   // arm DOWN = train stopped for orders
    const cls  = isStopped ? 'raised' : state === 'raised' ? 'lowered' : 'none';
    const sym  = state === 'raised' ? '▲' : '▼';

    let title, blocked = false;
    if (isStopped) {
        const ready = isSignalReadyToRaise(sid);
        if (!ready && (layoutRules.signal_reset_mode || 'hard') === 'hard') {
            blocked = true;
            title = 'Outstanding unacknowledged orders — cannot clear until all ACKed';
        } else {
            title = 'DOWN — stop for orders (click to raise/clear)';
        }
    } else {
        title = 'UP — clear (click to lower/activate for orders)';
    }

    return `<button class="arm-btn ${cls}${blocked ? ' blocked' : ''}" ` +
        `data-sid="${sid}" data-dir="${dir}" data-state="${state || 'raised'}" ` +
        `${blocked ? 'disabled ' : ''}title="${title}">${sym}</button>`;
}

function updateToSignal(sid, dir, state) {
    if (!_signalStates[sid]) _signalStates[sid] = {};
    _signalStates[sid][dir] = state;
    const el = document.getElementById(`to-${sid}-${dir}`);
    if (!el) return;
    el.innerHTML = armHtml(sid, dir, state);
}

function isSignalReadyToRaise(sid) {
    // Ready to clear (raise) when all TOs addressed to this station have been ACKed by it
    return toLogData.every(to =>
        !to.addressed_to.includes(sid) || (to.acks && to.acks[sid])
    );
}

function outstandingTosForStation(sid) {
    return toLogData.filter(to =>
        to.addressed_to.includes(sid) && !(to.acks && to.acks[sid])
    );
}

function updateAllSignalArms() {
    for (const sid of toSignalStations) {
        for (const dir of ['N', 'S']) {
            const state = (_signalStates[sid] || {})[dir] || 'raised';
            const el = document.getElementById(`to-${sid}-${dir}`);
            if (el) el.innerHTML = armHtml(sid, dir, state);
        }
    }
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

function fmtTime(t) {
    if (!t) return '';
    const [hStr, mStr] = t.split(':');
    const h = parseInt(hStr, 10), m = parseInt(mStr, 10);
    const ampm = h < 12 ? 'AM' : 'PM';
    const h12 = h % 12 || 12;
    return `${h12}:${String(m).padStart(2, '0')} ${ampm}`;
}

function updateNextTrain(sid, dir, train) {
    const el = document.getElementById(`nt-${sid}-${dir}`);
    if (!el) return;
    if (train) {
        const ar = train.arrive ? `<span class="train-arr">Ar&nbsp;${fmtTime(train.arrive)}</span>` : '';
        const dp = train.depart ? `<span class="train-dep">Dp&nbsp;${fmtTime(train.depart)}</span>` : '';
        const times = dir === 'N' ? ar + dp : dp + ar;
        el.innerHTML = `<span class="train-num">No.&nbsp;${train.number}</span><span class="train-times">${times}</span>`;
    } else {
        el.innerHTML = `<span class="train-none">—</span>`;
    }
}

// ── Signal arm command ────────────────────────────────────────────────────────

async function signalArmCmd(sid, dir, state) {
    try {
        await fetch('/api/signal/arm', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ station_id: sid, direction: dir, state }),
        });
    } catch {
        setStatus('Signal arm command failed', true);
    }
}

// ── OS log ───────────────────────────────────────────────────────────────────

function fmtRrTime(t) {
    if (!t || t === '?') return t || '?';
    const [hStr, mStr] = t.split(':');
    const h = parseInt(hStr, 10), m = parseInt(mStr, 10);
    const ampm = h < 12 ? 'AM' : 'PM';
    const h12 = h % 12 || 12;
    return `${h12}:${String(m).padStart(2, '0')} ${ampm}`;
}

function osEntryHtml(entry) {
    const prefix   = entry.work_extra ? 'WX' : entry.extra ? 'X' : '';
    const dirClass = entry.direction === 'N' ? 'os-dir-n' : 'os-dir-s';
    return `<div class="os-row">` +
        `<span class="os-col-time">${fmtRrTime(entry.rr_time)}</span>` +
        `<span class="os-col-sta">${entry.station_id}</span>` +
        `<span class="os-col-train ${dirClass}">${prefix}${entry.train}${entry.direction}</span>` +
        `</div>`;
}

function rebuildOsLog(entries) {
    const log = document.getElementById('os-log');
    log.innerHTML = entries.map(osEntryHtml).join('');
}

function handleOsReport(event) {
    const log = document.getElementById('os-log');
    const row = document.createElement('div');
    row.innerHTML = osEntryHtml(event.entry);
    const newRow = row.firstChild;
    newRow.classList.add('os-flash');
    log.prepend(newRow);
    while (log.children.length > 50) log.removeChild(log.lastChild);
}

// ── Clock controls ───────────────────────────────────────────────────────────

async function clockCmd(action, extra = {}) {
    try {
        await fetch('/api/clock/control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action, ...extra }),
        });
    } catch {
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

    document.getElementById('btn-to-issue').onclick = () => openToFormModal();
    document.getElementById('btn-to-form-close').onclick = () => closeToFormModal();
    document.getElementById('btn-to-submit').onclick = () => submitToOrder();
    document.getElementById('to-form-select').onchange = () =>
        validateToForm(document.getElementById('to-type-select').value);

    // Close modal on overlay click
    document.getElementById('modal-to-form').addEventListener('click', e => {
        if (e.target === e.currentTarget) closeToFormModal();
    });
}

// ── TO Issue modal ────────────────────────────────────────────────────────────

function populateToTypeSelect() {
    const sel = document.getElementById('to-type-select');
    sel.innerHTML = '';
    const types = toTypes.to_types || {};
    for (const [id, def] of Object.entries(types)) {
        const opt = document.createElement('option');
        opt.value = id;
        opt.textContent = def.label || id;
        sel.appendChild(opt);
    }
    sel.onchange = () => { buildToFields(sel.value); updateToPreview(); };
    if (sel.options.length > 0) buildToFields(sel.options[0].value);
}

function populateToStationChecks() {
    const container = document.getElementById('to-stations');
    container.innerHTML = '';
    for (const sid of stationIds) {
        const item = document.createElement('label');
        item.className = 'station-check-item';
        item.innerHTML = `<input type="checkbox" name="to-sta" value="${sid}"> ${sid}`;
        item.querySelector('input').onchange = (e) => {
            item.classList.toggle('checked', e.target.checked);
            updateToPreview();
        };
        container.appendChild(item);
    }
}

function populateFormSelect() {
    const sel = document.getElementById('to-form-select');
    const active = new Set(activeForms);
    // Keep placeholder; rebuild all form options
    sel.innerHTML = '<option value="" disabled selected>FORM ?</option>';
    for (const f of ALL_FORMS) {
        if (!active.has(f.letter)) continue;
        const opt = document.createElement('option');
        opt.value = f.letter;
        opt.textContent = `Form ${f.letter} — ${f.desc}`;
        sel.appendChild(opt);
    }
}

function buildToFields(toType) {
    const container = document.getElementById('to-fields');
    container.innerHTML = '';
    const typeDef = (toTypes.to_types || {})[toType];
    if (!typeDef) return;
    for (const f of typeDef.fields || []) {
        const row = document.createElement('div');
        row.className = 'form-row';
        row.dataset.fieldId = f.id;
        const label = document.createElement('label');
        label.setAttribute('for', `to-f-${f.id}`);
        label.textContent = f.label + (f.required ? '' : ' (optional)');
        row.appendChild(label);

        let input;
        if (f.type === 'boolean') {
            const wrap = document.createElement('div');
            wrap.className = 'form-bool';
            input = document.createElement('input');
            input.type = 'checkbox';
            input.id = `to-f-${f.id}`;
            if (f.default === true) input.checked = true;
            const lbl = document.createElement('label');
            lbl.setAttribute('for', `to-f-${f.id}`);
            lbl.textContent = f.label;
            wrap.appendChild(input);
            wrap.appendChild(lbl);
            row.innerHTML = '';  // clear label already added
            row.appendChild(wrap);
        } else if (f.type === 'direction') {
            input = document.createElement('select');
            input.id = `to-f-${f.id}`;
            input.innerHTML = '<option value="N">North (N)</option><option value="S">South (S)</option>';
            row.appendChild(input);
        } else if (f.type === 'station_id') {
            input = document.createElement('select');
            input.id = `to-f-${f.id}`;
            input.innerHTML = stationIds.map(s =>
                `<option value="${s}">${s} — ${stationNames[s] || s}</option>`).join('');
            row.appendChild(input);
        } else if (f.type === 'integer') {
            input = document.createElement('input');
            input.type = 'number';
            input.id = `to-f-${f.id}`;
            if (f.min != null) input.min = f.min;
            if (f.max != null) input.max = f.max;
            input.value = f.min ?? 2;
            row.appendChild(input);
        } else {
            // train_number, engine_number, string, rr_time
            input = document.createElement('input');
            input.type = 'text';
            input.id = `to-f-${f.id}`;
            if (f.type === 'rr_time') input.placeholder = 'HH:MM';
            row.appendChild(input);
        }

        if (f.help) {
            const hint = document.createElement('span');
            hint.className = 'hint';
            hint.textContent = f.help;
            row.appendChild(hint);
        }

        if (input) input.oninput = () => { handleFieldChange(toType); updateToPreview(); };
        if (input && input.type === 'select-one')
            input.onchange = () => { handleFieldChange(toType); updateToPreview(); };

        container.appendChild(row);
    }

    if (toType === 'meet') {
        const note = document.createElement('div');
        note.id = 'to-superiority';
        note.className = 'superiority-note hint-note';
        note.textContent = 'Select direction of Train A to verify superiority.';
        container.appendChild(note);
    }

    handleFieldChange(toType);
}

function handleFieldChange(toType) {
    // Hide/show fields with hidden_when conditions
    const typeDef = (toTypes.to_types || {})[toType];
    if (!typeDef) return;
    for (const f of typeDef.fields || []) {
        if (!f.hidden_when) continue;
        const row = document.querySelector(`[data-field-id="${f.id}"]`);
        if (!row) continue;
        let hide = false;
        for (const [condId, condVal] of Object.entries(f.hidden_when)) {
            const condEl = document.getElementById(`to-f-${condId}`);
            if (condEl && condEl.checked === condVal) { hide = true; break; }
        }
        row.style.display = hide ? 'none' : '';
    }
    validateToForm(toType);
    if (toType === 'meet') updateSuperiorityIndicator(getToFields(toType));
}

function validateToForm(toType) {
    const typeDef = (toTypes.to_types || {})[toType];
    const btn = document.getElementById('btn-to-submit');
    if (!typeDef) { btn.disabled = true; return; }
    const formSel = document.getElementById('to-form-select');
    if (!formSel || !formSel.value) { btn.disabled = true; return; }
    for (const f of typeDef.fields || []) {
        if (!f.required) continue;
        if (f.type === 'boolean') continue;
        const row = document.querySelector(`[data-field-id="${f.id}"]`);
        if (row && row.style.display === 'none') continue;
        const el = document.getElementById(`to-f-${f.id}`);
        if (!el || !el.value.trim()) { btn.disabled = true; return; }
    }
    // At least one station must be checked
    const checked = [...document.querySelectorAll('#to-stations input:checked')];
    btn.disabled = checked.length === 0;
}

function getToFields(toType) {
    const typeDef = (toTypes.to_types || {})[toType];
    if (!typeDef) return {};
    const result = {};
    for (const f of typeDef.fields || []) {
        const el = document.getElementById(`to-f-${f.id}`);
        if (!el) continue;
        if (f.type === 'boolean') result[f.id] = el.checked;
        else if (f.type === 'integer') result[f.id] = parseInt(el.value, 10) || 0;
        else result[f.id] = el.value.trim();
    }
    return result;
}

// ── TO text renderer ─────────────────────────────────────────────────────────
// Reads the template string from to_types.json and substitutes field values
// plus computed variables. Matches the CYD firmware rendering from the same fields.

function renderToText(toType, fields) {
    const typeDef = (toTypes.to_types || {})[toType];
    if (!typeDef || !typeDef.template) return '';

    const sn = sid => stationNames[sid] || sid || '?';
    const dw = d   => DIR_WORD[d] || d || '?';

    const computed = {
        station_name:      sn(fields.station),
        from_station_name: sn(fields.from_station),
        to_station_name:   sn(fields.to_station),
        direction_word:    dw(fields.direction),
        direction_b_word:  dw(fields.direction_b),
        train_b_ref:       fields.train_b_is_extra
            ? `Extra ${fields.train_b || '?'} ${dw(fields.direction_b)}`
            : `No. ${fields.train_b || '?'} Eng ${fields.engine_b || '?'}`,
        partial_suffix:    (fields.from_station && fields.to_station)
            ? ` ${sn(fields.from_station)} to ${sn(fields.to_station)}`
            : '',
    };

    const vars = Object.assign({}, fields, computed);
    return typeDef.template.replace(/\{(\w+)\}/g, (_, key) =>
        (vars[key] !== undefined && vars[key] !== null && String(vars[key]) !== '')
            ? String(vars[key])
            : '?'
    );
}

// ── Meet superiority indicator ────────────────────────────────────────────────

function updateSuperiorityIndicator(fields) {
    const el = document.getElementById('to-superiority');
    if (!el) return;

    const supDir = (layoutRules.superior_direction || '').toUpperCase();
    if (!supDir) { el.className = 'superiority-note'; el.textContent = ''; return; }

    const bExtra = !!fields.train_b_is_extra;
    const dirA   = (fields.direction_a || '').toUpperCase();

    if (bExtra) {
        el.className = 'superiority-note warn';
        el.textContent = 'Train B is extra — inferior to all scheduled trains. Train A (scheduled) should hold the main. Consider swapping.';
        return;
    }
    if (!dirA) {
        el.className = 'superiority-note hint-note';
        el.textContent = 'Select direction of Train A to verify superiority.';
        return;
    }
    if (dirA === supDir) {
        el.className = 'superiority-note warn';
        el.textContent = `Train A is ${DIR_WORD[dirA] || dirA} (superior direction) — should hold the main. Consider swapping Train A and B.`;
    } else {
        el.className = 'superiority-note ok';
        el.textContent = `✓ Train A is ${DIR_WORD[dirA] || dirA} (inferior direction) — correctly takes the siding.`;
    }
}

function updateToPreview() {
    const toType = document.getElementById('to-type-select').value;
    const fields = getToFields(toType);
    const text   = renderToText(toType, fields);
    document.getElementById('to-preview').textContent = text;
    validateToForm(toType);
}

async function submitToOrder() {
    const toType = document.getElementById('to-type-select').value;
    const form   = document.getElementById('to-form-select').value;
    const fields = getToFields(toType);
    const addressed_to = [...document.querySelectorAll('#to-stations input:checked')]
                          .map(el => el.value);
    const errEl = document.getElementById('to-issue-err');
    errEl.textContent = '';

    try {
        const r = await fetch('/api/to/issue', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ to_type: toType, form, fields, addressed_to }),
        });
        const body = await r.json();
        if (!r.ok) { errEl.textContent = body.error || 'Error'; return; }
        closeToFormModal();
    } catch {
        errEl.textContent = 'Request failed';
    }
}

function openToFormModal() {
    // Reset form to clean state
    document.getElementById('to-form-select').value = '';
    document.getElementById('to-issue-err').textContent = '';
    document.querySelectorAll('#to-stations input').forEach(el => {
        el.checked = false;
        el.closest('.station-check-item').classList.remove('checked');
    });
    const sel = document.getElementById('to-type-select');
    if (sel.options.length > 0) { buildToFields(sel.value); updateToPreview(); }
    validateToForm(sel.value);
    document.getElementById('modal-to-form').hidden = false;
}

function closeToFormModal() {
    document.getElementById('modal-to-form').hidden = true;
}

// ── TO Log (inline panel) ─────────────────────────────────────────────────────

function toLogEntryHtml(entry) {
    const typeDef = (toTypes.to_types || {})[entry.to_type] || {};
    const text = renderToText(entry.to_type, entry.fields || {});
    const trainAcks = entry.train_acks || {};
    const trainBadges = Object.entries(trainAcks).map(([train, rcvd]) =>
        `<span class="ack-badge train-badge ${rcvd ? 'received' : 'pending'}" title="${rcvd ? `Rcvd ${rcvd}` : 'Pending'}">No. ${train}</span>`
    ).join('');
    const stationBadges = Object.entries(entry.acks || {}).map(([sid, ack]) =>
        `<span class="ack-badge ${ack ? 'received' : 'pending'}" title="${ack ? `Station ACK ${ack.rr_time}` : 'Pending'}">${sid}</span>`
    ).join('');
    return `<div class="to-log-entry">
        <div class="to-log-header">
            <span class="to-log-seq">#${entry.seq}</span>
            <span class="to-log-type">${typeDef.label || entry.to_type}</span>
            <span class="to-log-time">${fmtRrTime(entry.issued_rr_time)}</span>
        </div>
        <div class="to-log-text">${escHtml(text)}</div>
        <div class="to-log-acks">${trainBadges}${stationBadges ? `<span class="ack-divider"></span>${stationBadges}` : ''}</div>
    </div>`;
}

function isToFullyAcked(to) {
    return to.addressed_to.length > 0 &&
           to.addressed_to.every(sid => to.acks && to.acks[sid]);
}

function refreshToLog() {
    const container = document.getElementById('to-log-list');
    if (toLogData.length === 0) {
        container.innerHTML = '<em class="muted">No orders issued.</em>';
        return;
    }
    const outstanding  = toLogData.filter(to => !isToFullyAcked(to));
    const acknowledged = toLogData.filter(to =>  isToFullyAcked(to));
    container.innerHTML =
        `<div class="to-log-section-header">Outstanding (${outstanding.length})</div>` +
        `<div class="to-log-section-body">${outstanding.length ? outstanding.map(toLogEntryHtml).join('') : '<em class="muted">None</em>'}</div>` +
        (acknowledged.length ? `<div class="to-log-section-header">Acknowledged (${acknowledged.length})</div>
        <div class="to-log-section-body">${acknowledged.map(toLogEntryHtml).join('')}</div>` : '');
}

// ── Utility ──────────────────────────────────────────────────────────────────

function setStatus(msg, isError = false) {
    const el = document.getElementById('status-bar');
    el.textContent = msg;
    el.className = isError ? 'error' : '';
}

function escHtml(s) {
    return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

// ── Boot ─────────────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
    initControls();
    connect();
    setInterval(_renderClockDisplay, 1000);
});
