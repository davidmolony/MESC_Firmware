/*
* Copyright 2021 cod3b453
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

var PROFILE_HEADER_ENTRIES = 32;
var neid = 0;
var eid = 0;

function init() {
    var app = window.document.getElementById('app');

    if (app == undefined) {
        alert('Failed to initialise');
        return;
    }

    app.innerHTML = '<div id="app-cmd">'
                  + '<h1>MESC Profile Tool</h1>'
                  + '<h2>Configuration</h2>'
                  + '<input type="button" id="app-btn-add" onclick="add()" value="Add"/> <input type="button" id="app-btn-gen" value="Generate"/>'
                  + '<hr/>'
                  + '<div id="app-list"></div>'
                  + '</div>'
                  + '<hr/>'
                  + '<h2>Generation</h2>'
                  + '<label>Version:</label> ' + '1' + '.' + '0' + '<br />'
                  + '<label>Timestamp:</label> ' + TIMESTAMP + '<br />'
                  + '<label>git hash:</label> ' + GITHASH + '<br/>'
                  + '<textarea readonly></textarea>';
}

function add() {
    if (neid >= PROFILE_HEADER_ENTRIES) {
        alert("Entry limit (" + PROFILE_HEADER_ENTRIES + ") reached");
        return;
    }

    var app_list = window.document.getElementById('app-list');

    var eN = document.createElement('div');
    eN.className = 'entrybox';
    eN.id = 'e-' + eid;

    var l = document.createElement('label');
    l.for = 'e-' + eid;
    l.innerHTML = 'Type: ';

    eN.appendChild( l );

    var s = document.createElement('select');
    s.id = 'e-' + eid + '-type';

    let types = [ '- Select -', 'Battery', 'Speed', 'Temperature' ];

    for ( i=0; i < types.length; i++ ) {
        var o = document.createElement('option');
        o.value = i;
        o.innerHTML = types[i];
        s.appendChild( o );
    }

    var og = document.createElement('optgroup');
    og.label = 'UI'

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
    r.innerHTML = '<input type="button" id="app-btn-rem-' + eid + '" value="Remove"/>';
    r.addEventListener( 'click', function() { rem(this) }, false );

    eN.appendChild( r );

    app_list.appendChild( eN );

    neid = neid + 1;
    eid = eid + 1;
}

function makeLabelledAddress(obj,id,label,value,change_fn,unit) {
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + eid + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + eid + '-' + id;
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
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + eid + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + eid + '-' + id;
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
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + eid + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('input');

    inp.id = 'e-' + eid + '-' + id;
    inp.type = 'number';
    inp.value = value;

    obj.appendChild( inp );

    inp.addEventListener( 'change', change_fn, false );

    var unt = document.createElement('select');
    unt.id = 'e-' + eid + '-' + id + '-unit';
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

function makeOptions(obj,id,label,value,unit) {
    var lbl = document.createElement('label');

    lbl.innerHTML = label;
    lbl.setAttribute( 'for', 'e-' + eid + '-' + id );

    obj.appendChild( lbl );

    var inp = document.createElement('select');
    inp.id = 'e-' + eid + '-' + id;
    inp.value = value;
    for ( i = 0; i < unit.length; i++ ) {
        var opt = document.createElement('option');
        opt.value = i;
        opt.innerHTML = unit[i];
        inp.appendChild( opt );
    }
    obj.appendChild( inp );

    obj.appendChild( document.createElement('br') );

    return inp;
}

function plotBattery(obj) {
    var eid = obj.id.split('-')[1];
    var obj_graph = window.document.getElementById('e-' + eid + '-bat-graph');
    var obj_ctx = obj_graph.getContext('2d');

    var obj_imax = window.document.getElementById('e-' + eid + '-cell-imax');
    var obj_vmax = window.document.getElementById('e-' + eid + '-cell-vmax');
    var obj_cmax = window.document.getElementById('e-' + eid + '-cell-cmax');

    var obj_vmid = window.document.getElementById('e-' + eid + '-cell-vmid');
    var obj_cmid = window.document.getElementById('e-' + eid + '-cell-cmid');

    var obj_vlow = window.document.getElementById('e-' + eid + '-cell-vlow');
    var obj_clow = window.document.getElementById('e-' + eid + '-cell-clow');

    var obj_vmin = window.document.getElementById('e-' + eid + '-cell-vmin');

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

    var obj_p = window.document.getElementById('e-' + eid + '-bat-p');
    var obj_s = window.document.getElementById('e-' + eid + '-bat-s');

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

    var obj_i = window.document.getElementById('e-' + eid + '-bat-imax');

    var i = parseInt(obj_i.value);

    var pmax = (s * vmax * i);

    var obj_pmax = window.document.getElementById('e-' + eid + '-bat-pmax');

    // TODO pmax

}

function makeBattery(eN) {
    var o = document.createElement('div');

    var c = document.createElement('canvas');
    c.id = 'e-' + eid + '-bat-graph';
    c.width = 400;
    c.height = 400;
    o.appendChild( c );

    // Cell
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Cell';
    o.appendChild( h3 );

    var inp = makeLabelledNumber( o, 'cell-imax', 'Imax', 30, function() { plotBattery(this) }, 'A' );
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

    inp = makeLabelledNumber( o, 'bat-s', 'Serial', 20, function() { plotBattery(this) }, '' );
    inp.min = 1;

    inp = makeOptions( o, 'bat-disp', 'Display', 0, ['Percent (%)','Amp-Hour (Ah)'] );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Regeneration';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-regen' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-regen';
    inp.type = 'checkbox';
    o.appendChild( inp );

    eN.appendChild( o );

    plotBattery( c );
}

function plotSpeed(obj) {

}

function makeSpeed(eN) {
    var o = document.createElement('div');

    // Motor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Motor';
    o.appendChild( h3 );

    var inp = makeLabelledNumber( o, 'spd-imax', 'Imax', 60, function() { plotSpeed(this) }, 'A' );
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
    lbl.setAttribute( 'for', 'e-' + eid + '-spd-dir' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-spd-dir';
    inp.type = 'checkbox';
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Regeneration';
    lbl.setAttribute( 'for', 'e-' + eid + '-spd-regen' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-spd-regen';
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
        lbl.setAttribute( 'for', 'e-' + eid + '-spd-hall' + i );

        o.appendChild( lbl );

        inp = document.createElement('input');

        inp.id = 'e-' + eid + '-spd-hall' + i;
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
}

function plotTemperature(obj) {
    var eid = obj.id.split('-')[1];
    var obj_graph = window.document.getElementById('e-' + eid + '-temp-graph');
    var obj_ctx = obj_graph.getContext('2d');

    var obj_v    = window.document.getElementById('e-' + eid + '-temp-v');
    var obj_rf   = window.document.getElementById('e-' + eid + '-temp-rf');

    var obj_sch  = window.document.getElementById('e-' + eid + '-temp-schema');

    var obj_adcr = window.document.getElementById('e-' + eid + '-temp-adcrng');

    var obj_beta = window.document.getElementById('e-' + eid + '-temp-beta');
    var obj_r    = window.document.getElementById('e-' + eid + '-temp-r');

    var obj_t0   = window.document.getElementById('e-' + eid + '-temp-t0');

    var obj_min  = window.document.getElementById('e-' + eid + '-temp-min');
    var obj_max  = window.document.getElementById('e-' + eid + '-temp-max');

    var V         = parseFloat(obj_v.value);
    var R_F       = parseFloat(obj_rf.value);
    var schema    =            obj_sch.value;
    var adc_range = parseFloat(obj_adcr.value);
    var beta      = parseFloat(obj_beta.value);
    var r         = parseFloat(obj_r.value);
    var T0        = parseFloat(obj_t0.value);
    var tmin      = parseFloat(obj_min.value);
    var tmax      = parseFloat(obj_max.value);

    var adc_min = adc_range;
    var adc_max = 0;

    var ldx = 51;
    var ldy = 0;

    if (schema == 0) { // TEMP_SCHEMA_R_F_ON_R_T
        ldy = 75;
    } else if (schema == 1) { // TEMP_SCHEMA_R_T_ON_R_F
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

        if (schema == 0) {
            vout = (V * R_T) / (R_F + R_T);
        } else if (schema == 1) {
            vout = (V * R_F) / (R_F + R_T);
        }

        var adc_raw = ((vout * adc_range) / V);

        if (T == 0) {
            if (schema == 0) {
                adc_min = adc_raw;
            } else if (schema == 1) {
                adc_max = adc_raw;
                adc_min = adc_range - adc_raw;
            }

            obj_ctx.fillText( (adc_raw).toFixed(0), 20,  ldy );
            obj_ctx.fillText( '(' + (vout).toFixed(1) + 'V)', 20,  ldy + 10);
        }

        var dx = (T * 250) / tmax;
        var dy = 0;

        if (schema == 0) {
            dy = ((adc_min - adc_raw) * 250) / adc_min;
        } else if (schema == 1) {
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
}

function makeTemperature(eN) {
    var o = document.createElement('div');

    var c = document.createElement('canvas');
    c.id = 'e-' + eid + '-temp-graph';
    c.width = 400;
    c.height = 400;
    o.appendChild( c );

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    var inp = makeLabelledNumber( o, 'temp-v', 'V', 3.3, function() { plotTemperature(this) }, 'V' );
    inp.min = 1.0;
    inp.step = 0.1;

    inp = makeLabelledNumber( o, 'temp-rf', 'R_F', 4700, function() { plotTemperature(this) }, '&Omega;' );
    inp.min = 1;

    inp = makeOptions( o, 'temp-schema', 'Schematic', 0, ['R_F on R_T','R_T on R_F'] );

    inp = makeLabelledNumber( o, 'temp-adc', 'ADC', 3, function() { plotTemperature(this) }, '' );
    inp.min = 0;
    inp.step = 1;
    inp.max = 4;

    inp = makeLabelledNumber( o, 'temp-adcrng', 'ADC Range', 4096, function() { plotTemperature(this) }, '' );

    inp = makeLabelledNumber( o, 'temp-t0', 'T0', 25, function() { plotTemperature(this) }, '&deg;C' );

    inp = makeLabelledNumber( o, 'temp-r0', 'R0', 10000, function() { plotTemperature(this) }, '&Omega;' );

    inp = makeLabelledNumber( o, 'temp-beta', 'Beta', 3438, function() { plotTemperature(this) }, '&Omega;' );
    inp.min = 1;

    inp = makeLabelledNumber( o, 'temp-r', 'r', 0.0982, function() { plotTemperature(this) }, '' );

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

    plotTemperature( c );
}

function makeThrottle(eN) {
    var o = document.createElement('div');

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    var inp = makeOptions( o, 'thr-if', 'Interface', 0, ['?','??'] );

    inp = makeLabelledNumber( o, 'thr-adc-min', 'ADC min', 100, function() { plotThrottle(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'thr-adc-max', 'ADC max', 2047, function() { plotThrottle(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'thr-rsp', 'Response', 0, ['Linear','Logarithmic'] );

    eN.appendChild( o );
}

function makeBrake(eN) {
    var o = document.createElement('div');

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Sensor';
    o.appendChild( h3 );

    var inp = makeLabelledNumber( o, 'brk-adc-min', 'ADC min', 100, function() { plotBrake(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'brk-adc-max', 'ADC max', 2047, function() { plotBrake(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'brk-rsp', 'Response', 0, ['Linear','Logarithmic'] );


    eN.appendChild( o );
}

function makeButton(eN) {
    var o = document.createElement('div');

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    var inp = makeOptions( o, 'btn-if', 'Interface', 0, ['?','??'] );

    inp = makeLabelledAddress( o, 'btn-a', 'Address', '80001000', function() { plotButton(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'btn-id', 'Id', 0, function() { plotButton(this) }, '' );
    inp.min = 0;

    eN.appendChild( o );
}

function makeIndicator(eN) {
    var o = document.createElement('div');

    // Sensor
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    var inp = makeOptions( o, 'ind-if', 'Interface', 0, ['?','??'] );

    inp = makeLabelledAddress( o, 'ind-a', 'Address', '80002000', function() { plotInd(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'ind-id', 'Id', 0, function() { plotInd(this) }, '' );
    inp.min = 0;

    inp = makeOptions( o, 'ind-act', 'Activation', 0, ['Edge','Level'] );

    eN.appendChild( o );
}

function plotScreen(obj) {
    var eid = obj.id.split('-')[1];
    var obj_s = window.document.getElementById('e-' + eid + '-scrn'  );
    var obj_w = window.document.getElementById('e-' + eid + '-scrn-w');
    var obj_h = window.document.getElementById('e-' + eid + '-scrn-h');

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
}

function makeScreen(eN) {
    var o = document.createElement('div');

    // Interface
    var h3 = document.createElement('h3');
    h3.innerHTML = 'Interface';
    o.appendChild( h3 );

    var inp = makeOptions( o, 'scrn-if', 'Interface', 0, ['?','??'] );

    inp = makeLabelledAddress( o, 'scrn-a', 'Address', '80003000', function() { plotScreen(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'scrn-w', 'Width', 30, function() { plotScreen(this) }, '' );
    inp.min = 0;

    inp = makeLabelledNumber( o, 'scrn-h', 'Height', 6, function() { plotScreen(this) }, '' );
    inp.min = 0;

    eN.appendChild( o );

    // Interface
    h3 = document.createElement('h3');
    h3.innerHTML = 'Preview';
    o.appendChild( h3 );

    var c = document.createElement('div');
    c.id = 'e-' + eid + '-scrn';
    c.className = 'screen';
    o.appendChild( c );

    plotScreen( c );
}

function setType(obj) {
    if (obj.value == 0) {
        return;
    }

    obj.disabled = true;

    var eN = obj.parentNode;

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
    eN.parentNode.removeChild( eN );
    neid = neid - 1;
}
