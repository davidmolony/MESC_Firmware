/*
* Copyright 2021-2022 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

var header = new ProfileHeader();
var neid = 0;

function updateFavIcon() {
    var favicon = document.getElementById('favicon');

    var link = document.createElement('link');
    link.id = 'favicon';
    link.rel = 'shortcut icon';

    var canvas = document.createElement('canvas');
    canvas.width  = 32;
    canvas.height = 32;

    var context = canvas.getContext('2d');

    context.beginPath();
    context.rect( 0, 0, canvas.width, canvas.height );
    context.fillStyle = "white";
    context.fill();

    context.beginPath();
    context.strokeStyle = '#0080FF';
    context.moveTo( 4, 16 );

    for ( var i=1; i<24; ) {
        var y = 12 - (8 * Math.sin( (2 * Math.PI * i) / 24 ));
        context.lineTo( 4 + i, y );
        i++;
        if (i < 24) {
            context.moveTo( 4 + i, y );
        }
    }

    context.stroke();

    context.font = '12px monospace';
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillStyle = '#000000';
    context.fillText( "MESC", 16, 24 );

    if (neid > 0) {
        context.beginPath();
        context.arc( canvas.width - canvas.width / 3 , canvas.height / 3, 32 / 3, 0, 2 * Math.PI );
        context.fillStyle = '#FF0000';
        context.fill();

        var fontsize = '18';

        if (neid > 9) {
            fontsize = '16';
        }

        context.font = 'bold ' + fontsize + 'px "helvetica", sans-serif';
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillStyle = '#FFFFFF';
        context.fillText( neid, canvas.width - canvas.width  / 3, 1 + canvas.height / 3 );
    }

    link.href = canvas.toDataURL('image/png');

    if (favicon) {
        document.head.removeChild( favicon );
    }

    document.head.appendChild( link );
}

function init() {
    var app = window.document.getElementById('app');

    if (app == undefined) {
        alert('Failed to initialise');
        return;
    }

    app.innerHTML = '<div id="app-cmd">'
                  + '<h1>MESC Profile Tool</h1>'
                  + '<h2>Configuration</h2>'
                  + '<input type="button" id="app-btn-add" onclick="add()" value="Add"/> <input type="button" id="app-btn-gen" onclick="gen()" value="Generate"/>'
                  + '<hr/>'
                  + '<div id="app-list"></div>'
                  + '</div>'
                  + '<hr/>'
                  + '<h2>Generation</h2>'
                  + '<label>Version:</label> ' + '1' + '.' + '0' + '<br />'
                  + '<label>Timestamp:</label> ' + TIMESTAMP + '<br />'
                  + '<label>git hash:</label> ' + GITHASH + '<br/>'
                  + '<textarea id="app-prf-dmp" readonly></textarea>';
}

function add() {
    if (neid >= PROFILE_HEADER_ENTRIES) {
        alert("Entry limit (" + PROFILE_HEADER_ENTRIES + ") reached");
        return;
    }

    var app_list = window.document.getElementById('app-list');

    var entry_index = neid; // TODO replace header.findFreeEntry()

    var eN = document.createElement('div');
    eN.className = 'entrybox';
    eN.id = 'e-' + entry_index;

    var l = document.createElement('label');
    l.for = 'e-' + entry_index;
    l.innerHTML = 'Type: ';

    eN.appendChild( l );

    var s = document.createElement('select');
    s.id = 'e-' + entry_index + '-type';

    let types = [ '- Select -', 'Battery', 'Speed', 'Temperature' ];

    for ( i=0; i < types.length; i++ ) {
        var o = document.createElement('option');
        o.value = i;
        o.innerHTML = types[i];
        s.appendChild( o );
    }

    var og = document.createElement('optgroup');
    og.label = 'UI';

    let uitypes = [ 'Throttle', 'Brake', 'Button', 'Indicator', 'Screen' ];

    for ( i=0; i < uitypes.length; i++ ) {
        var o = document.createElement('option');
        o.value = types.length + i;
        o.innerHTML = uitypes[i];
        og.appendChild( o );
    }

    s.appendChild( og );

    s.addEventListener( 'change', function() { setType(this) }, false );

    eN.appendChild( s );

    var r = document.createElement('span');
    r.className = 'action';
    r.innerHTML = '<input type="button" id="app-btn-rem-' + entry_index + '" value="Remove"/>';
    r.addEventListener( 'click', function() { rem(this) }, false );

    eN.appendChild( r );

    app_list.appendChild( eN );

    neid = neid + 1;

    updateFavIcon();
}

function updateName(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var entry = header.getEntry( entry_index );
    entry.setName( obj.value );
}

function makeLabelledText(obj,id,label, value) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + entry_index + '-' + id );

    obj.appendChild( lbl );

   var inp = document.createElement('input');

    inp.id = 'e-' + entry_index + '-' + id;
    inp.type = 'text';
    inp.value = value;
    inp.className = 'name';

    obj.appendChild( inp );

    inp.addEventListener( 'change', function() { updateName(this) }, false );

    obj.appendChild( document.createElement('br') );

    return inp;
}

function makeLabelledAddress(obj,id,label,value,change_fn,unit) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + entry_index + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + entry_index + '-' + id;
    inp.type = 'text';
    inp.value = value;

    obj.appendChild( inp );

    inp.addEventListener( 'change', change_fn, false );

    var unt = document.createElement('span');
    unt.innerHTML = unit;

    obj.appendChild( unt );
    obj.appendChild( document.createElement('br') );

    return inp;
}

function makeLabelledNumber(obj,id,label,value,change_fn,unit) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + entry_index + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + entry_index + '-' + id;
    inp.type = 'number';
    inp.value = value;

    obj.appendChild( inp );

    inp.addEventListener( 'change', change_fn, false );

    var unt = document.createElement('span');
    unt.innerHTML = unit;

    obj.appendChild( unt );
    obj.appendChild( document.createElement('br') );

    return inp;
}

function makeLabelledNumberUnit(obj,id,label,value,change_fn,unit) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + entry_index + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + entry_index + '-' + id;
    inp.type = 'number';
    inp.value = value;

    obj.appendChild( inp );

    inp.addEventListener( 'change', change_fn, false );

    var unt = document.createElement('select');
    unt.id = 'e-' + entry_index + '-' + id + '-unit';
    for ( i = 0; i < unit.length; i++ ) {
        var opt = document.createElement('option');
        opt.value = i;
        opt.innerHTML = unit[i];
        unt.appendChild( opt );
    }
    obj.appendChild( unt );
    obj.appendChild( document.createElement('br') );

    return inp;
}

function makeOptions(obj,id,label,value,unit,change_fn) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + entry_index + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('select');
    inp.id = 'e-' + entry_index + '-' + id;
    inp.value = value;
    for ( i = 0; i < unit.length; i++ ) {
        var opt = document.createElement('option');
        opt.value = i;
        opt.innerHTML = unit[i];
        inp.appendChild( opt );
    }
    obj.appendChild( inp );
    inp.addEventListener( 'change', change_fn, false );

    obj.appendChild( document.createElement('br') );

    return inp;
}

function plotBattery(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_graph = window.document.getElementById('e-' + entry_index + '-bat-graph');
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_ctx = obj_graph.getContext('2d');

    var obj_imax = window.document.getElementById('e-' + entry_index + '-cell-imax');
    var obj_vmax = window.document.getElementById('e-' + entry_index + '-cell-vmax');
    var obj_cmax = window.document.getElementById('e-' + entry_index + '-cell-cmax');

    var obj_vmid = window.document.getElementById('e-' + entry_index + '-cell-vmid');
    var obj_cmid = window.document.getElementById('e-' + entry_index + '-cell-cmid');

    var obj_vlow = window.document.getElementById('e-' + entry_index + '-cell-vlow');
    var obj_clow = window.document.getElementById('e-' + entry_index + '-cell-clow');

    var obj_vmin = window.document.getElementById('e-' + entry_index + '-cell-vmin');

    obj_ctx.clearRect( 0, 0, obj_graph.width, obj_graph.height );

    // Y-axis
    obj_ctx.strokeStyle = "#000000";
    obj_ctx.setLineDash([]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 50,  50 );
        obj_ctx.lineTo( 50, 350 );
    obj_ctx.stroke();

    var vmax = parseFloat(obj_vmax.value);
    var vmid = parseFloat(obj_vmid.value);
    var vlow = parseFloat(obj_vlow.value);
    var vmin = parseFloat(obj_vmin.value);

    var vrange = vmax - vmin;

    obj_ctx.fillText( vmax, 20,  75 );
    obj_ctx.fillText( vmin, 20, 350 );
    obj_ctx.fillText( 'V', 45, 40 );

    // X-axis
    obj_ctx.beginPath();
        obj_ctx.moveTo(  50, 350 );
        obj_ctx.lineTo( 350, 350 );
    obj_ctx.stroke();

    var cmax = parseInt(obj_cmax.value);
    var cmid = parseInt(obj_cmid.value);
    var clow = parseInt(obj_clow.value);
    var cmin = 0;

    var crange = cmax;

    obj_ctx.fillText( cmax,  40, 360 );
    obj_ctx.fillText( cmin, 325, 360 );
    obj_ctx.fillText( 'mAh', 355, 355 );

    // Max-Mid
    var dy = (vmax - vmid) * 250 / vrange;
    var dx = (cmax - cmid) * 250 / crange;

    obj_ctx.strokeStyle = "#808080";
    obj_ctx.setLineDash([5,5]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51, 75 + dy );
        obj_ctx.lineTo( 51 + dx, 75 + dy );
    obj_ctx.stroke();
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51 + dx, 75 + dy );
        obj_ctx.lineTo( 51 + dx, 349 );
    obj_ctx.stroke();

    obj_ctx.strokeStyle = "#00FF00";
    obj_ctx.setLineDash([]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51,  75 );
        obj_ctx.lineTo( 51 + dx, 75 + dy );
    obj_ctx.stroke();

    obj_ctx.fillText( vmid, 20, 75 + dy );
    obj_ctx.fillText( cmid, 41 + dx, 360 );

    // Mid-Low
    var ldy = dy;
    var ldx = dx;

    dy = (vmax - vlow) * 250 / vrange;
    dx = (cmax - clow) * 250 / crange;

    obj_ctx.strokeStyle = "#808080";
    obj_ctx.setLineDash([5,5]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51, 75 + dy );
        obj_ctx.lineTo( 51 + dx, 75 + dy );
    obj_ctx.stroke();
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51 + dx, 75 + dy );
        obj_ctx.lineTo( 51 + dx, 349 );
    obj_ctx.stroke();

    obj_ctx.strokeStyle = "#FF7F00";
    obj_ctx.setLineDash([]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51 + ldx, 75 + ldy );
        obj_ctx.lineTo( 51 + dx, 75 + dy );
    obj_ctx.stroke();

    obj_ctx.fillText( vlow, 20, 75 + dy );
    obj_ctx.fillText( clow, 41 + dx, 375 );

    // Low-Min
    obj_ctx.strokeStyle = "#FF0000";
    obj_ctx.setLineDash([]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 51 + dx, 75 + dy );
        obj_ctx.lineTo( 325, 349 );
    obj_ctx.stroke();

    //

    var obj_p = window.document.getElementById('e-' + entry_index + '-bat-p');
    var obj_s = window.document.getElementById('e-' + entry_index + '-bat-s');

    var p = parseInt(obj_p.value);
    var s = parseInt(obj_s.value);

    var chalf = cmax / 2;
    // if Chalf > Cmid
    var crem = chalf - cmid;
    var dv = vmax - vmid;
    var vnom = ((vmid + ((crem * dv) / cmax)) * s).toFixed(1);
    var cnom = (p * ((cmax - clow) / 1000));
    var pnom = (vnom * cnom);
    var suffix = '';

    if (pnom > 1000) {
        pnom = (pnom / 1000).toFixed(1);
        suffix = 'k';
    } else {
        pnom = (pnom).toFixed(0);
    }

    obj_ctx.fillText( 'Vmax ' + (vmax * s) + ' V' ,  300, 10 );
    obj_ctx.fillText( 'Vnom ' +  vnom      + ' V' ,  300, 20 );
    obj_ctx.fillText( 'Vlow ' + (vlow * s) + ' V' ,  300, 30 );
    obj_ctx.fillText( 'Vmin ' + (vmin * s) + ' V' ,  300, 40 );
    obj_ctx.fillText( 'Cnom ' +  cnom      + ' Ah',  300, 50 );
    obj_ctx.fillText( 'Pnom ' +  pnom + ' ' + suffix + 'Wh',  300, 60 );

    var obj_i = window.document.getElementById('e-' + entry_index + '-bat-imax');

    var i = parseInt(obj_i.value);

    var pmax = (s * vmax * i);

    var obj_pmax = window.document.getElementById('e-' + entry_index + '-bat-pmax');

    // TODO pmax

    var obj_esr = window.document.getElementById('e-' + entry_index + '-bat-esr');

    var obj_disp = window.document.getElementById('e-' + entry_index + '-bat-disp');
    var obj_regen = window.document.getElementById('e-' + entry_index + '-bat-regen');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.cell_Imax = parseFloat(obj_imax.value);
    entry._profile.cell_Vmax = parseFloat(obj_vmax.value);
    entry._profile.cell_Cmax = parseFloat(obj_cmax.value) / 1000.0;

    entry._profile.cell_Vmid = parseFloat(obj_vmid.value);
    entry._profile.cell_Cmid = parseFloat(obj_cmid.value) / 1000.0;

    entry._profile.cell_Vlow = parseFloat(obj_vlow.value);
    entry._profile.cell_Clow = parseFloat(obj_clow.value) / 1000.00;

    entry._profile.cell_Vmin = parseFloat(obj_vmin.value);

    entry._profile.battery_Imax = parseFloat(obj_i.value);
    entry._profile.battery_Pmax = parseFloat(obj_pmax.value);

    entry._profile.battery_ESR = parseFloat(obj_esr.value) / 1000.00;

    entry._profile.battery_parallel = parseInt(obj_p.value);
    entry._profile.battery_series = parseInt(obj_s.value);

    entry._profile.display = parseInt(obj_disp.value);
    entry._profile.allow_regen = (obj_regen.checked ? 1 : 0);
}

function makeBattery(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var c = document.createElement('canvas');
    c.id = 'e-' + entry_index + '-bat-graph';
    c.width = 400;
    c.height = 400;
    o.appendChild( c );

    var inp = makeLabelledText( o, 'name', 'Name', 'Battery' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Cell
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Cell';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'cell-imax', 'Imax', 30, function() { plotBattery(this) }, 'A' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'cell-vmax', 'Vmax', 4.20, function() { plotBattery(this) }, 'V' );
    inp.min = 1;
    inp.step = 0.01;

    inp = makeLabelledNumber( o, 'cell-cmax', 'Cmax', 4200, function() { plotBattery(this) }, 'mAh' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'cell-vmid', 'Vmid', 3.4, function() { plotBattery(this) }, 'V' );
    inp.min = 1;
    inp.step = 0.01;

    inp = makeLabelledNumber( o, 'cell-cmid', 'Cmid', 700, function() { plotBattery(this) }, 'mAh' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'cell-vlow', 'Vlow', 3.2, function() { plotBattery(this) }, 'V' );
    inp.min = 1;
    inp.step = 0.01;

    inp = makeLabelledNumber( o, 'cell-clow', 'Clow', 500, function() { plotBattery(this) }, 'mAh' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'cell-vmin', 'Vmin', 2.8, function() { plotBattery(this) }, 'V' );
    inp.min = 0;
    inp.step = 0.01;

    // Battery
    h3 = document.createElement('h3');
    h3.innerHTML = 'Battery';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'bat-imax', 'Imax', 50, function() { plotBattery(this) }, 'A' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'bat-pmax', 'Pmax', 250, function() { plotBattery(this) }, 'W' );

    inp = makeLabelledNumber( o, 'bat-esr', 'ESR', 100, function() { plotBattery(this) }, 'm&Omega;' );

    inp = makeLabelledNumber( o, 'bat-p', 'Parallel', 2, function() { plotBattery(this) }, '' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'bat-s', 'Series', 20, function() { plotBattery(this) }, '' );
    inp.min = 1;

    inp = makeOptions( o, 'bat-disp', 'Display', 0, ['Percent (%)','Amp-Hour (Ah)'] );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Regeneration';
    lbl.setAttribute( 'for', 'e-' + entry_index + '-bat-regen' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + entry_index + '-bat-regen';
    inp.type = 'checkbox';
    o.appendChild( inp );

    eN.appendChild( o );

    entry = header.addEntry( entry_index, BAT_PROFILE_SIGNATURE );
    entry._profile = new BATProfile();

    plotBattery( c );
}

function plotSpeed(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_imax = window.document.getElementById('e-' + entry_index + '-spd-imax');
    var obj_vmax = window.document.getElementById('e-' + entry_index + '-spd-vmax');
    var obj_pmax = window.document.getElementById('e-' + entry_index + '-spd-pmax');
    var obj_rpmmax = window.document.getElementById('e-' + entry_index + '-spd-rpmmax');
    var obj_pp = window.document.getElementById('e-' + entry_index + '-spd-pp');
    var obj_dir = window.document.getElementById('e-' + entry_index + '-spd-dir');
    var obj_regen = window.document.getElementById('e-' + entry_index + '-spd-regen');
    var obj_hall = new Array(6);
    for (var h = 0; h < 6; h++ ) {
    obj_hall[h] = window.document.getElementById('e-' + entry_index + '-spd-hall' + h);
    }
    var obj_gm = window.document.getElementById('e-' + entry_index + '-spd-gm');
    var obj_gw = window.document.getElementById('e-' + entry_index + '-spd-gw');
    var obj_d = window.document.getElementById('e-' + entry_index + '-spd-d');
    var obj_disp = window.document.getElementById('e-' + entry_index + '-spd-disp');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.motor_Imax = parseFloat(obj_imax.value);
    entry._profile.motor_Vmax = parseFloat(obj_vmax.value);
    entry._profile.motor_Pmax = parseFloat(obj_pmax.value);
    entry._profile.motor_RPMmax = parseInt(obj_rpmmax.value);
    entry._profile.motor_pole_pairs = parseInt(obj_pp.value);
    entry._profile.motor_direction = (obj_dir.checked ? 1 : 0);
    entry._profile.motor_allow_regen = (obj_regen.checked ? 1 : 0);
    entry._profile.sensor_type = /*undefined*/0;
    for ( var h = 0; h < 6; h++ ) {
    entry._profile.sensor_hall_states[h] = parseInt(obj_hall[h].value);
    }
    entry._profile.gear_ratio_motor = parseInt(obj_gm.value);
    entry._profile.gear_ratio_wheel = parseInt(obj_gw.value);
    entry._profile.wheel_diameter = parseFloat(obj_d.value);
    entry._profile.wheel_conversion = parseFloat(obj_disp.value);
}

function makeSpeed(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'Speed' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Motor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Motor';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'spd-imax', 'Imax', 60, function() { plotSpeed(this) }, 'A' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'spd-vmax', 'Vmax', 100, function() { plotSpeed(this) }, 'V' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'spd-pmax', 'Pmax', 250, function() { plotSpeed(this) }, 'W' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'spd-rpmmax', 'RPMmax', 250, function() { plotSpeed(this) }, '' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'spd-pp', 'Pole Pairs', 6, function() { plotSpeed(this) }, '' );
    inp.min = 4;

    lbl = document.createElement('label');
    lbl.innerHTML = 'Direction';
    lbl.setAttribute( 'for', 'e-' + entry_index + '-spd-dir' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + entry_index + '-spd-dir';
    inp.type = 'checkbox';
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Regeneration';
    lbl.setAttribute( 'for', 'e-' + entry_index + '-spd-regen' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + entry_index + '-spd-regen';
    inp.type = 'checkbox';
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    // Sensor
    h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    // type

    for ( i = 0; i < 6; i++ ) {
        lbl = document.createElement('label');

        lbl.innerHTML = 'Hall ' + i;
        lbl.setAttribute( 'for', 'e-' + entry_index + '-spd-hall' + i );

        o.appendChild( lbl );

        inp = document.createElement('input');

        inp.id = 'e-' + entry_index + '-spd-hall' + i;
        inp.type = 'number';
        inp.min = 0;
        inp.value = (0 + ((65535 / 7) * i)).toFixed(0);
        inp.max = 65535;
        inp.step = 1;

        o.appendChild( inp );

        inp.addEventListener( 'change', function() { plotSpeed(this) }, false );

        o.appendChild( document.createElement('br') );
    }

    // Trasmission
    h3 = document.createElement('h3');
    h3.innerHTML = 'Transmission';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'spd-gm', 'Gear (Motor)', 1, function() { plotSpeed(this) }, '' );

    inp = makeLabelledNumber( o, 'spd-gw', 'Gear (Wheel)', 1, function() { plotSpeed(this) }, '' );

    // Wheel & Display
    h3 = document.createElement('h3');
    h3.innerHTML = 'Wheel &amp; Display';
    o.appendChild( h3 );

    inp = makeLabelledNumberUnit( o, 'spd-d', 'Diameter', 26, function() { plotSpeed(this) }, ['in','cm'] );
    inp.min = 1;
    inp.step = 0.1;

    inp = makeOptions( o, 'spd-disp', 'Display', 0, ['mph','kph'] );

    eN.appendChild( o );

    entry = header.addEntry( entry_index, SPEED_PROFILE_SIGNATURE );
    entry._profile = new SPEEDProfile();

    plotSpeed( inp );
}

function plotTemperature(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_graph = window.document.getElementById('e-' + entry_index + '-temp-graph');
    var obj_name  = window.document.getElementById('e-' + entry_index + '-name');
    var obj_ctx   = obj_graph.getContext('2d');

    var obj_v    = window.document.getElementById('e-' + entry_index + '-temp-v');
    var obj_rf   = window.document.getElementById('e-' + entry_index + '-temp-rf');
    // TODO adc
    var obj_adcr = window.document.getElementById('e-' + entry_index + '-temp-adcrng');
    // TODO method
    var obj_sch  = window.document.getElementById('e-' + entry_index + '-temp-schema');
    // TODO A/B/Tlo
    // TODO A/B/C
    var obj_beta = window.document.getElementById('e-' + entry_index + '-temp-beta');
    var obj_r    = window.document.getElementById('e-' + entry_index + '-temp-r');

    var obj_t0   = window.document.getElementById('e-' + entry_index + '-temp-t0');
    var obj_r0   = window.document.getElementById('e-' + entry_index + '-temp-r0');

    var obj_min  = window.document.getElementById('e-' + entry_index + '-temp-min');
    var obj_max  = window.document.getElementById('e-' + entry_index + '-temp-max');

    var V         = parseFloat(obj_v.value);
    var R_F       = parseFloat(obj_rf.value);
    // TODO adc
    var adc_range = parseInt(  obj_adcr.value);
    // TODO method
    var schema    = parseInt(  obj_sch.value);
    // TODO A/B/Tlo
    // TODO A/B/C
    var beta      = parseFloat(obj_beta.value);
    var r         = parseFloat(obj_r.value);

    var T0        = parseFloat(obj_t0.value);
    var R0        = parseFloat(obj_r0.value);

    var tmin      = parseFloat(obj_min.value);
    var tmax      = parseFloat(obj_max.value);

    var adc_min = adc_range;
    var adc_max = 0;

    var ldx = 51;
    var ldy = 0;

    if (schema == TEMP_SCHEMA_R_F_ON_R_T) {
        ldy = 75;
    } else if (schema == TEMP_SCHEMA_R_T_ON_R_F) {
        ldy = 75 + 250;
    }

    obj_ctx.clearRect( 0, 0, obj_graph.width, obj_graph.height );

    // Y-axis
    obj_ctx.strokeStyle = "#000000";
    obj_ctx.setLineDash([]);
    obj_ctx.beginPath();
        obj_ctx.moveTo( 50,  50 );
        obj_ctx.lineTo( 50, 350 );
    obj_ctx.stroke();

    obj_ctx.fillText( 'ADC', 45, 40 );

    // X-axis
    obj_ctx.beginPath();
        obj_ctx.moveTo(  50, 350 );
        obj_ctx.lineTo( 350, 350 );
    obj_ctx.stroke();

    obj_ctx.fillText( '0' ,  40, 360 );
    obj_ctx.fillText( 'T' , 355, 355 );

    // Curve
    for ( T = 0; T < (tmax + 10); T++ ) {
        var K    = 273.15 + T;
        var R_T  = r * Math.exp( beta / K );
        var vout = 0;

        if (schema == TEMP_SCHEMA_R_F_ON_R_T) {
            vout = (V * R_T) / (R_F + R_T);
        } else if (schema == TEMP_SCHEMA_R_T_ON_R_F) {
            vout = (V * R_F) / (R_F + R_T);
        }

        var adc_raw = ((vout * adc_range) / V);

        if (T == 0) {
            if (schema == TEMP_SCHEMA_R_F_ON_R_T) {
                adc_min = adc_raw;
            } else if (schema == TEMP_SCHEMA_R_T_ON_R_F) {
                adc_max = adc_raw;
                adc_min = adc_range - adc_raw;
            }

            obj_ctx.fillText( (adc_raw).toFixed(0), 20,  ldy );
            obj_ctx.fillText( '(' + (vout).toFixed(1) + 'V)', 20,  ldy + 10);
        }

        var dx = (T * 250) / tmax;
        var dy = 0;

        if (schema == TEMP_SCHEMA_R_F_ON_R_T) {
            dy = ((adc_min - adc_raw) * 250) / adc_min;
        } else if (schema == TEMP_SCHEMA_R_T_ON_R_F) {
            dy = 250 - (((adc_raw - adc_max) * 250) / adc_min);
        }

        if ((T == tmin) || (T == T0) || (T == tmax)) {
            obj_ctx.strokeStyle = "#808080";
            obj_ctx.setLineDash([5,5]);
            obj_ctx.beginPath();
                obj_ctx.moveTo( 51, ldy );
                obj_ctx.lineTo( ldx, ldy );
            obj_ctx.stroke();
            obj_ctx.beginPath();
                obj_ctx.moveTo( ldx, ldy );
                obj_ctx.lineTo( ldx, 349 );
            obj_ctx.stroke();

            obj_ctx.fillText( (adc_raw).toFixed(0), 20,  ldy + 10 );
            obj_ctx.fillText( '(' + (vout).toFixed(1) + 'V)', 20,  ldy + 20);
            obj_ctx.fillText( (T).toFixed(0), ldx, 360 );
        }

        if (T != 0) {
            obj_ctx.strokeStyle = "#800080";
            obj_ctx.setLineDash([]);

            obj_ctx.beginPath();
                obj_ctx.moveTo( ldx,  ldy );
                ldx = 51 + dx;
                ldy = 75 + dy;
                obj_ctx.lineTo( ldx,  ldy );
            obj_ctx.stroke();
        }
    }

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.V = V;
    entry._profile.R_F = R_F;
    // TODO adc
    entry._profile.adc_range = adc_range;

    entry._profile.method = /*undefined*/TEMP_METHOD_STEINHART_HART_BETA_R/*TEMP_METHOD_STEINHART_HART_ABC*/;
    entry._profile.schema = schema;

    switch (entry._profile.method) {
        case TEMP_METHOD_CURVE_APPROX:
            entry._profile.parameters_approx_A = undefined;
            entry._profile.parameters_approx_B = undefined;

            entry._profile.parameters_approx_Tlo = undefined;
            break;
        case TEMP_METHOD_STEINHART_HART_ABC:
        case TEMP_METHOD_STEINHART_HART_BETA_R:
            entry._profile.parameters_SH_A = undefined;
            entry._profile.parameters_SH_B = undefined;
            entry._profile.parameters_SH_C = undefined;

            entry._profile.parameters_SH_Beta = beta;
            entry._profile.parameters_SH_r    = r;

            entry._profile.parameters_SH_T0 = T0;
            entry._profile.parameters_SH_R0 = R0;

            TEMP_derive_SteinhartHart_ABC_from_Beta( entry._profile )
            break;
    }

    entry._profile.limit_Tmin = tmin;
    entry._profile.limit_Tmax = tmax;
}

function makeTemperature(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var c = document.createElement('canvas');
    c.id = 'e-' + entry_index + '-temp-graph';
    c.width = 400;
    c.height = 400;
    o.appendChild( c );

    var inp = makeLabelledText( o, 'name', 'Name', 'Temperature' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'temp-v', 'V', 3.3, function() { plotTemperature(this) }, 'V' );
    inp.min = 1.0;
    inp.step = 0.1;

    inp = makeLabelledNumber( o, 'temp-rf', 'R_F', 4700, function() { plotTemperature(this) }, '&Omega;' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'temp-adc', 'ADC', 3, function() { plotTemperature(this) }, '' );
    inp.min = 0;
    inp.step = 1;
    inp.max = 4;

    inp = makeLabelledNumber( o, 'temp-adcrng', 'ADC Range', 4096, function() { plotTemperature(this) }, '' );
    // TODO method
    inp = makeOptions( o, 'temp-schema', 'Schematic', 0, ['R_F on R_T','R_T on R_F'], function() { plotTemperature(this) } );
    // TODO A/B/Tlo
    // TODO A/B/C
    inp = makeLabelledNumber( o, 'temp-beta', 'Beta', 3438, function() { plotTemperature(this) }, '&Omega;' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'temp-r', 'r', 0.0982, function() { plotTemperature(this) }, '' );

    inp = makeLabelledNumber( o, 'temp-t0', 'T0', 25, function() { plotTemperature(this) }, '&deg;C' );

    inp = makeLabelledNumber( o, 'temp-r0', 'R0', 10000, function() { plotTemperature(this) }, '&Omega;' );

    // Limit
    h3 = document.createElement('h3');
    h3.innerHTML = 'Limit';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'temp-min', 'Tmin', 10, function() { plotTemperature(this) }, '&deg;C' );
    inp.min = 0;
    inp.max = 20;

    inp = makeLabelledNumber( o, 'temp-max', 'Tmax', 50, function() { plotTemperature(this) }, '&deg;C' );
    inp.min = 20;
    inp.max = 100;

    eN.appendChild( o );

    entry = header.addEntry( entry_index, TEMP_PROFILE_SIGNATURE );
    entry._profile = new TEMPProfile();

    plotTemperature( c );
}

function plotThrottle(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_adc_min  = window.document.getElementById('e-' + entry_index + '-thr-adc-min');
    var obj_adc_max  = window.document.getElementById('e-' + entry_index + '-thr-adc-max');
    var obj_resp = window.document.getElementById('e-' + entry_index + '-thr-rsp');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.throttle_adc_min  = parseInt(obj_adc_min.value);
    entry._profile.throttle_adc_max  = parseInt(obj_adc_max.value);
    entry._profile.throttle_response = parseInt(obj_resp.value);
}

function makeThrottle(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'Throttle' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    inp = makeOptions( o, 'thr-if', 'Interface', 0, ['?','??'], function() { plotThrottle(this) } );

    inp = makeLabelledNumber( o, 'thr-adc-min', 'ADC min', 100, function() { plotThrottle(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'thr-adc-max', 'ADC max', 2047, function() { plotThrottle(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'thr-rsp', 'Response', 0, ['Linear','Logarithmic'], function() { plotThrottle(this) } );

    eN.appendChild( o );

    entry = header.addEntry( entry_index, UI_PROFILE_SIGNATURE );
    entry._profile = new UIProfile( UI_PROFILE_THROTTLE );

    plotThrottle( inp );
}

function plotBrake(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_adc_min  = window.document.getElementById('e-' + entry_index + '-brk-adc-min');
    var obj_adc_max  = window.document.getElementById('e-' + entry_index + '-brk-adc-max');
    var obj_resp = window.document.getElementById('e-' + entry_index + '-brk-rsp');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.brake_adc_min  = parseInt(obj_adc_min.value);
    entry._profile.brake_adc_max  = parseInt(obj_adc_max.value);
    entry._profile.brake_response = parseInt(obj_resp.value);
}

function makeBrake(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'E-Brake' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    inp = makeLabelledNumber( o, 'brk-adc-min', 'ADC min', 100, function() { plotBrake(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'brk-adc-max', 'ADC max', 2047, function() { plotBrake(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'brk-rsp', 'Response', 0, ['Linear','Logarithmic'], function() { plotBrake(this) } );


    eN.appendChild( o );

    entry = header.addEntry( entry_index, UI_PROFILE_SIGNATURE );
    entry._profile = new UIProfile( UI_PROFILE_BRAKE );

    plotBrake( inp );
}

function plotButton(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_if   = window.document.getElementById('e-' + entry_index + '-btn-if');
    var obj_addr = window.document.getElementById('e-' + entry_index + '-btn-a');
    var obj_id   = window.document.getElementById('e-' + entry_index + '-btn-id');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.button_interface  = parseInt(obj_if.value);
    entry._profile.button_address    = parseInt(obj_addr.value);
    entry._profile.button_identifier = parseInt(obj_id.value);
}

function makeButton(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'Button' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    inp = makeOptions( o, 'btn-if', 'Interface', 0, ['?','??'], function() { plotButton(this) } );

    inp = makeLabelledAddress( o, 'btn-a', 'Address', '80001000', function() { plotButton(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'btn-id', 'Id', 0, function() { plotButton(this) }, '' );
    inp.min = 0;

    eN.appendChild( o );

    entry = header.addEntry( entry_index, UI_PROFILE_SIGNATURE );
    entry._profile = new UIProfile( UI_PROFILE_BUTTON );

    plotButton( inp );
}

function plotIndicator(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_if   = window.document.getElementById('e-' + entry_index + '-ind-if');
    var obj_addr = window.document.getElementById('e-' + entry_index + '-ind-a');
    var obj_id   = window.document.getElementById('e-' + entry_index + '-ind-id');
    var obj_act  = window.document.getElementById('e-' + entry_index + '-ind-act');

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.indicator_interface  = parseInt(obj_if.value);
    entry._profile.indicator_address    = parseInt(obj_addr.value);
    entry._profile.indicator_identifier = parseInt(obj_id.value);
    entry._profile.indicator_activation = parseInt(obj_act.value);
}

function makeIndicator(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'Indicator' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    inp = makeOptions( o, 'ind-if', 'Interface', 0, ['?','??'], function() { plotIndicator(this) } );

    inp = makeLabelledAddress( o, 'ind-a', 'Address', '80002000', function() { plotIndicator(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'ind-id', 'Id', 0, function() { plotIndicator(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'ind-act', 'Activation', 0, ['Edge','Level'], function() { plotIndicator(this) } );

    eN.appendChild( o );

    entry = header.addEntry( entry_index, UI_PROFILE_SIGNATURE );
    entry._profile = new UIProfile( UI_PROFILE_INDICATOR );

    plotIndicator( inp );
}

function plotScreen(obj) {
    var entry_index = parseInt(obj.id.split('-')[1]);
    var obj_name = window.document.getElementById('e-' + entry_index + '-name');
    var obj_s = window.document.getElementById('e-' + entry_index + '-scrn'  );
    var obj_if = window.document.getElementById('e-' + entry_index + '-scrn-if');
    var obj_a = window.document.getElementById('e-' + entry_index + '-scrn-a');
    var obj_w = window.document.getElementById('e-' + entry_index + '-scrn-w');
    var obj_h = window.document.getElementById('e-' + entry_index + '-scrn-h');

    var w = parseFloat(obj_w.value);
    var h = parseFloat(obj_h.value);

    var s = '';

    s = s + '<table>';

    for ( var y = 0; y < h; y++ )
    {
        s = s + '<tr>';
        for ( var x = 0; x < w; x++ )
        {
            s = s + '<td>&nbsp;</td>';
        }
        s = s + '</tr>';
    }

    s = s + '</table>';

    obj_s.innerHTML = s;

    updateName( obj_name );

    var entry = header.getEntry( entry_index );

    entry._profile.screen_interface = parseInt(obj_if.value);
    entry._profile.screen_address   = parseInt(obj_a.value);
    entry._profile.screen_width     = w;
    entry._profile.screen_height    = h;
}

function makeScreen(eN) {
    var entry_index = parseInt(eN.id.split('-')[1]);
    var o = document.createElement('div');
    o.id = 'e-' + entry_index + '-disp';

    var inp = makeLabelledText( o, 'name', 'Name', 'Screen' );
    inp.maxLength = PROFILE_ENTRY_MAX_NAME_LENGTH;

    // Interface
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    inp = makeOptions( o, 'scrn-model', 'Model', 0, ['SH1106 (128x64)'], function() { plotScreen(this) } );

    inp = makeOptions( o, 'scrn-if', 'Interface', 0, ['?','??'], function() { plotScreen(this) } );

    inp = makeLabelledAddress( o, 'scrn-a', 'Address', '80003000', function() { plotScreen(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'scrn-w', 'Width', 16, function() { plotScreen(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'scrn-h', 'Height', 4, function() { plotScreen(this) }, '' );
    inp.min = 0;

    eN.appendChild( o );

    // Interface
    h3 = document.createElement('h3');
    h3.innerHTML = 'Preview';
    o.appendChild( h3 );

    var c = document.createElement('div');
    c.id = 'e-' + entry_index + '-scrn';
    c.className = 'screen';
    o.appendChild( c );

    entry = header.addEntry( entry_index, UI_PROFILE_SIGNATURE );
    entry._profile = new UIProfile( UI_PROFILE_SCREEN );

    plotScreen( c );
}

function setType(obj) {
    if (obj.value == 0) {
        return;
    }

    obj.disabled = true;

    var eN = obj.parentNode;
    var entry_index = parseInt(obj.id.split('-')[1]);

    switch (obj.value) {
        case '1':
            makeBattery( eN );
            break;
        case '2':
            makeSpeed( eN );
            break;
        case '3':
            makeTemperature( eN );
            break;
        case '4':
            makeThrottle( eN );
            break;
        case '5':
            makeBrake( eN );
            break;
        case '6':
            makeButton( eN );
            break;
        case '7':
            makeIndicator( eN );
            break;
        case '8':
            makeScreen( eN );
            break;
    }
}

function rem(obj) {
    var eN = obj.parentNode;
    var entry_index = parseInt(eN.id.split('-')[1]);
    header.remEntry( entry_index );
    eN.parentNode.removeChild( eN );
    neid = neid - 1;
    updateFavIcon();
}

function gen()
{
    console.clear();
    var prf_dmp = window.document.getElementById('app-prf-dmp');

    // TODO populate header

    prf_dmp.value = dump_ProfileHeader( header );
}