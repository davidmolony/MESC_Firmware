
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

function plotBattery(obj) {
    var eid = obj.id.split('-')[1]
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
    var cnom = (p * s * ((cmax - clow) / 1000));
    var pnom = ((vnom * cnom) / s);
    var suffix = '';

    if (pnom > 1000) {
        pnom = (pnom / 1000).toFixed(1);
        suffix = 'k';
    } else {
        pnom = (pnom).toFixed(0);
    }

    obj_ctx.fillText( 'Vnom ' + vnom + ' V' ,  300, 10 );
    obj_ctx.fillText( 'Cnom ' + cnom + ' Ah',  300, 20 );
    obj_ctx.fillText( 'Pnom ' + pnom + ' ' + suffix + 'Wh',  300, 30 );

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

    // Imax
    var lbl = document.createElement('label');
    lbl.innerHTML = 'Imax';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-imax' );
    o.appendChild( lbl );
    var inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-imax';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 30;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    var unit = document.createElement('span');
    unit.innerHTML = 'A';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Vmax Cmax
    lbl = document.createElement('label');
    lbl.innerHTML = 'Vmax';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-vmax' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-vmax';
    inp.type = 'number';
    inp.min = 1;
    inp.step = 0.01;
    inp.value = 4.20;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'V';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Cmax';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-cmax' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-cmax';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 4200;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'mAh';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Vmid Cmid
    lbl = document.createElement('label');
    lbl.innerHTML = 'Vmid';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-vmid' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-vmid';
    inp.type = 'number';
    inp.min = 1;
    inp.step = 0.01;
    inp.value = 3.40;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'V';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Cmid';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-cmid' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-cmid';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 700;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'mAh';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Vlow Clow
    lbl = document.createElement('label');
    lbl.innerHTML = 'Vlow';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-vlow' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-vlow';
    inp.type = 'number';
    inp.min = 1;
    inp.step = 0.01;
    inp.value = 3.20;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'V';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Clow';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-clow' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-clow';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 500;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'mAh';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Vmin
    lbl = document.createElement('label');
    lbl.innerHTML = 'Vmin';
    lbl.setAttribute( 'for', 'e-' + eid + '-cell-vmin' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-cell-vmin';
    inp.type = 'number';
    inp.min = 1;
    inp.step = 0.01;
    inp.value = 2.80;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotBattery(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'V';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Battery
    h3 = document.createElement('h3');
    h3.innerHTML = 'Battery';
    o.appendChild( h3 );

    // Imax
    lbl = document.createElement('label');
    lbl.innerHTML = 'Imax';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-imax' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-imax';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 50;
    o.appendChild( inp );
    unit = document.createElement('span');
    unit.innerHTML = 'A';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Pmax
    lbl = document.createElement('label');
    lbl.innerHTML = 'Pmax';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-pmax' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-pmax';
    inp.type = 'text';
    inp.value = 250;
    o.appendChild( inp );
    unit = document.createElement('span');
    unit.innerHTML = 'W';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // ESR
    lbl = document.createElement('label');
    lbl.innerHTML = 'ESR';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-esr' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-esr';
    inp.type = 'number';
    inp.value = 100;
    o.appendChild( inp );
    unit = document.createElement('span');
    unit.innerHTML = 'm&Omega;';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // Parallel
    // Series
    lbl = document.createElement('label');
    lbl.innerHTML = 'Parallel';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-p' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-p';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 2;
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    lbl = document.createElement('label');
    lbl.innerHTML = 'Series';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-s' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-bat-s';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 20;
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    // Display %/Ah
    lbl = document.createElement('label');
    lbl.innerHTML = 'Display';
    lbl.setAttribute( 'for', 'e-' + eid + '-bat-disp' );
    o.appendChild( lbl );
    inp = document.createElement('select');
    inp.id = 'e-' + eid + '-bat-disp';
    var opt = document.createElement('option');
    opt.value = 0;
    opt.innerHTML = 'Percent (%)';
    inp.appendChild(opt);
    opt = document.createElement('option');
    opt.value = 1;
    opt.innerHTML = 'Amp-Hour (Ah)';
    inp.appendChild(opt);
    o.appendChild( inp );
    o.appendChild( document.createElement('br') );

    // Allow regeneration
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

function makeSpeed(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Motor
    //   Imax
    //   Vmax
    //   Pmax
    //   RPMmax
    //   Pole pairs
    //   Direction
    //   Allow regeneration
    // Sensor
    //   Type
    //   Hall States [6]
    // Transmission
    //   Ratio (Motor : Wheel)
    // Wheel/Display
    // Diameter : Unit
    // Display (Unit, Conversion)
    eN.appendChild( o );
}

function plotTemperature(obj) {
    var eid = obj.id.split('-')[1]
    var obj_graph = window.document.getElementById('e-' + eid + '-temp-graph');
    var obj_ctx = obj_graph.getContext('2d');

    var obj_v    = window.document.getElementById('e-' + eid + '-temp-v');
    var obj_rf   = window.document.getElementById('e-' + eid + '-temp-rf');

    var obj_adcr = window.document.getElementById('e-' + eid + '-temp-adc');

    var obj_beta = window.document.getElementById('e-' + eid + '-temp-beta');
    var obj_r    = window.document.getElementById('e-' + eid + '-temp-r');

    var obj_t0   = window.document.getElementById('e-' + eid + '-temp-t0');

    var obj_min  = window.document.getElementById('e-' + eid + '-temp-min');
    var obj_max  = window.document.getElementById('e-' + eid + '-temp-max');

    var V         = parseFloat(obj_v.value);
    var R_F       = parseFloat(obj_rf.value);
    var adc_range = parseFloat(obj_adcr.value);
    var beta      = parseFloat(obj_beta.value);
    var r         = parseFloat(obj_r.value);
    var T0        = parseFloat(obj_t0.value);
    var tmin      = parseFloat(obj_min.value);
    var tmax      = parseFloat(obj_max.value);

    var adc_min = adc_range;
    var adc_max = 0;

    var ldx = 51;
    var ldy = 75;

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
        var vout = (V * R_T) / (R_F + R_T);
        var adc_raw = ((vout * adc_range) / V);

        if (T == 0) {
            adc_min = adc_raw;
            obj_ctx.fillText( (adc_min).toFixed(0), 20,  75 );
            obj_ctx.fillText( '(' + (vout).toFixed(1) + 'V)', 20,  75 + 10);
        }

        var dx = (T * 250) / tmax;
        var dy = ((adc_min - adc_raw) * 250) / adc_min;

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

    // V
    lbl = document.createElement('label');
    lbl.innerHTML = 'V';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-v' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-v';
    inp.type = 'number';
    inp.min = 1.0;
    inp.value = 3.3;
    inp.step = 0.1;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = 'V';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    //   R_F
    lbl = document.createElement('label');
    lbl.innerHTML = 'R_F';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-rf' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-rf';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 4700;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = '&Omega;';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    //   ADC range
    lbl = document.createElement('label');
    lbl.innerHTML = 'ADC Range';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-adc' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-adc';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 4096;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    o.appendChild( document.createElement('br') );

    // T0
    lbl = document.createElement('label');
    lbl.innerHTML = 'T0';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-t0' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-t0';
    inp.type = 'number';
    inp.value = 25;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = '&deg;C';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    // R0
    lbl = document.createElement('label');
    lbl.innerHTML = 'R0';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-r0' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-r0';
    inp.type = 'number';
    inp.value = 10000;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = '&Omega;';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    //     Beta
    lbl = document.createElement('label');
    lbl.innerHTML = 'Beta';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-beta' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-beta';
    inp.type = 'number';
    inp.min = 1;
    inp.value = 3438;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    o.appendChild( document.createElement('br') );

    //     r
    lbl = document.createElement('label');
    lbl.innerHTML = 'r';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-r' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-r';
    inp.type = 'number';
    inp.value = 0.0982;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    o.appendChild( document.createElement('br') );

    // Limit
    h3 = document.createElement('h3');
    h3.innerHTML = 'Limit';
    o.appendChild( h3 );

    //   Tmin
    lbl = document.createElement('label');
    lbl.innerHTML = 'Tmin';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-min' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-min';
    inp.type = 'number';
    inp.min = -40;
    inp.max = 100;
    inp.value = 10;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = '&deg;C';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    //   Tmax
    lbl = document.createElement('label');
    lbl.innerHTML = 'Tmax';
    lbl.setAttribute( 'for', 'e-' + eid + '-temp-max' );
    o.appendChild( lbl );
    inp = document.createElement('input');
    inp.id = 'e-' + eid + '-temp-max';
    inp.type = 'number';
    inp.min = -40;
    inp.max = 100;
    inp.value = 50;
    o.appendChild( inp );
    inp.addEventListener( 'change', function() { plotTemperature(this) }, false );
    unit = document.createElement('span');
    unit.innerHTML = '&deg;C';
    o.appendChild( unit );
    o.appendChild( document.createElement('br') );

    eN.appendChild( o );

    plotTemperature( c );
}

function makeThrottle(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Sensor
    //   Interface {ADC,Serial}
    //   ADC min
    //   ADC max
    //   Response (LIN,LOG}
    eN.appendChild( o );
}

function makeBrake(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Sensor
    //   ADC min
    //   ADC max
    //   Response (LIN,LOG}
    eN.appendChild( o );
}

function makeButton(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Interface
    // Address
    // Identifier
    eN.appendChild( o );
}

function makeIndicator(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Interface
    // Address
    // Identifier
    // Activation
    eN.appendChild( o );
}

function makeScreen(eN) {
    var o = document.createElement('div');
    o.innerHTML = '(Currently unsupported)';
    // Interface
    // Address
    // Width
    // Height
    eN.appendChild( o );
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
