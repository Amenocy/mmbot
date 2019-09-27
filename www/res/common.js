"use strict";


function beginOfDay(dt) {
	return new Date(dt.getFullYear(), dt.getMonth(), dt.getDate());
}

 
function fetch_json(file, opt) {
	return fetch(file, opt).then(function(req) {
		if (req.status>299 || req.status < 200) throw req;
		return req.json();
	});
}

function invPrice(v, inv) {
	return inv?1/v:v;
}
function invSize(v, inv) {
	return (inv?-1:1)*v;
}

function adjNum(n, decimals) {
	if (typeof n != "number") return n;	
	if (isNaN(n)) return "---";
	if (!isFinite(n)) { 
		if (s < 0) return "-∞";
		else return "∞";
	}
	if (decimals !== undefined) return n.toFixed(decimals);
	var an = Math.abs(n);
	if (an >= 100000) return n.toFixed(0);
	else if (an >= 1) return n.toFixed(2);
	else if (an > 0.0001) return n.toFixed(6);
	else {
		var s = (n*1000000).toFixed(3);
		if (s == "0.000") return s;
		return s+"µ";
	}
}

function new_svg_el(name, attrs, childOf) {
	var elem = document.createElementNS("http://www.w3.org/2000/svg", name);
	if (attrs) {for (var i in attrs) {
		elem.setAttributeNS(null,i, attrs[i]);
	}}
	if (childOf) childOf.appendChild(elem);
	return elem;
}

function pow2(x) {
	return x*x;
}

function calculateBacktest(params) {

var data = params.data;
var ea = params.external_assets;
var ga = params.sliding_pos;
var fb = params.fade;
var mlt = params.multiplicator || 1;
var mos = params.min_order_size || 0;
var inv = params.invert || false;
var et = params.expected_trend || 0;
var pos = (inv?-1:1)*params.start_pos || 0;


if (data.length == 0) return {
	pl:0,
	pln: 0,
	pos: 0,
	mdd:0,
	maxpos:0,
	eq:0,
	chart:[]
};
var pp = inv?1/data[0].price:data[0].price;
var eq = pp*pow2((ea+pos)/ea);
var pl = 0;
var mdd = 0;
var mpos = 0;
var norm = 0;
var tframe = ga*3600000;
var tm = data[0].time;
var newchart = [];

data.forEach(function(x) {
	var p = inv?1/x.price:x.price;
    var dr = Math.sign(p - pp);
	var pldiff = pos * (p - pp);
	var tmdiff = x.time - tm;
	if (tmdiff > Math.abs(tframe)) tmdiff = Math.abs(tframe);
	var neq = pldiff*Math.sign(tframe)>0?eq + (p - eq) * (tmdiff/Math.abs(tframe)):eq;
	if (Math.abs(pos) > mpos) mpos = Math.abs(pos);			
	var nxpos = ea*Math.sqrt(eq*(1+et)/p)-ea;
	var dpos = nxpos - pos ;
	var mult = (fb?(fb - Math.abs(nxpos))/fb:1)*mlt;
	if (mult < 0.000001) mult = 0.000001
	if (pos * dr < 0) {
		mult = (mult * 3+1)/4;
	}
	if (dpos * dr >= -mos) {
		if (mos <= 0) return;
		dpos = -mos * dr
	}
	pos = pos + dpos * mult;
	pp = p;
	eq = neq;
	tm = x.time;
	pl = pl+pldiff;
	norm = pl+pos*(eq-Math.sqrt(p*eq));
	if (pl < mdd) mdd = pl;
	newchart.push({
		price:x.price,
		pl:pl,
		pos:pos,
		pln:norm,
		np:inv?1/eq:eq,
		time:x.time,
		achg:(inv?-1:1)*dpos
	});
});
if (inv) eq = 1/eq;

return {
	pl:pl,
	pln: norm,
	pos: (inv?-1:1)*pos,
	mdd:mdd,
	maxpos:mpos,
	eq:eq,
	chart:newchart
};
}


function initChart(chart_interval, ratio, base_interval) {

	if (!ratio) ratio = 3;
	if (!base_interval) base_interval=1200000;




 return function (elem, chart, fld, lines, fld2) {
	"use strict";

	elem.innerText = "";
	if (chart.length == 0) return;
	
	var now = new Date(chart[chart.length-1].time);
	var bday = beginOfDay(now);		
	var skiphours = Math.floor((now - bday)/(base_interval));
			
	
	var daystep = 24*3600000/base_interval;
	var step = 2*864000000/chart_interval;
	var activewidth=step*chart_interval/base_interval;
	var dattextstep = Math.ceil(120/(daystep*step));
	var activeheight = (activewidth/ratio)|0;
	var minmax = chart.concat(lines?lines:[]).reduce(function(c,x) {
		if (c.min > x[fld]) c.min = x[fld];
		if (c.max < x[fld]) c.max = x[fld];
		return c;
	},{min:chart[0][fld],max:chart[0][fld]});
	minmax.sz = minmax.max - minmax.min;
	minmax.min =  minmax.min - minmax.sz*0.05;
	minmax.max =  minmax.max + minmax.sz*0.05;
	var priceStep = activeheight/(minmax.max-minmax.min);
	var axis = 2;
	var label = 20;
	var rowstep = Math.pow(10,Math.floor(Math.log10((minmax.max-minmax.min)/3)));
	var rowbeg= Math.floor(minmax.min/rowstep);
	var rowend= Math.floor(minmax.max/rowstep);	

	var svg = new_svg_el("svg",{viewBox:"0 0 "+(activewidth+axis)+" "+(activeheight+axis+label)},elem);
	new_svg_el("line",{x1:axis, y1:0,x2:axis,y2:activeheight+axis,class:"lineaxis"},svg);
	new_svg_el("line",{x1:0, y1:activeheight+1,x2:activewidth+axis,y2:activeheight+1,class:"lineaxis"},svg);
	var cnt = chart.length;
	function map_y(p) {
		return activeheight-(p-minmax.min)*priceStep;
	}
	function map_x(x) {
		return x*step+axis;
	}

	if (!isFinite(rowbeg) || !isFinite(rowend)) return;
	
	for (var i = rowbeg; i <=rowend; i++) {
		var v = i*rowstep;
		var maj = Math.abs(v)<rowstep/2;
		var y = map_y(v);
		new_svg_el("line",{x1:0,y1:y,x2:activewidth+axis,y2:y,class:maj?"majoraxe":"minoraxe"},svg);
		new_svg_el("text",{x:axis+2,y:y,class:"textaxis"},svg).appendChild(document.createTextNode(adjNum(i*rowstep)));
	}
	
	var xtmpos = activewidth/step-skiphours;
	while (xtmpos > 0) {
		var xtm = map_x(xtmpos);
		new_svg_el("line",{x1:xtm,y1:0,x2:xtm,y2:activeheight,class:"minoraxe"},svg);
		xtmpos-=daystep;
	}
	xtmpos = activewidth/step-skiphours;
	while (xtmpos > 0) {
		var xtm = map_x(xtmpos);
		new_svg_el("text",{x:xtm,y:activeheight,class:"textaxisx"},svg).appendChild(document.createTextNode(bday.toLocaleDateString()));
		bday.setDate(bday.getDate()-dattextstep);
		new_svg_el("line",{x1:xtm,y1:activeheight-5,x2:xtm,y2:activeheight,class:"majoraxe"},svg);
		xtmpos-=daystep*dattextstep;
	}
	var tmstart=(now/base_interval-activewidth/step)
	for (var i = 0; i <cnt-1; i++) {
		var pos = "stdline";
		var x1 = map_x(chart[i].time/base_interval-tmstart);
		var x2 = map_x(chart[i+1].time/base_interval-tmstart);
		var y1 = map_y(chart[i][fld]);
		var y2 = map_y(chart[i+1][fld]);
		new_svg_el("line",{x1:x1,y1:y1,x2:x2,y2:y2,class:pos},svg);
	}
	if (fld2) {
		for (var i = 0; i <cnt-1; i++) {
			var pos = "stdline2";
			var x1 = map_x(chart[i].time/base_interval-tmstart);
			var x2 = map_x(chart[i+1].time/base_interval-tmstart);
			var y1 = map_y(chart[i][fld2]);
			var y2 = map_y(chart[i+1][fld2]);
			new_svg_el("line",{x1:x1,y1:y1,x2:x2,y2:y2,class:pos},svg);
		}
	}
	for (var i = 0; i <cnt; i++) if (chart[i].achg) {
		var x1 = map_x(chart[i].time/base_interval-tmstart);
		var y1 = map_y(chart[i][fld]);
		var man = chart[i].man;
		var marker = "marker "+(chart[i].achg<0?"sell":"buy") 
		if (man) {
			new_svg_el("line",{x1:x1-5,y1:y1-5,x2:x1+5,y2:y1+5,class:marker},svg);
			new_svg_el("line",{x1:x1+5,y1:y1-5,x2:x1-5,y2:y1+5,class:marker},svg);
		} else {				
			new_svg_el("circle",{cx:x1,cy:y1,r:4,class:marker},svg);
		}
	}
	
	if (lines === undefined) {
		lines=[];
	}
	var l = {};
	l[fld] = chart[chart.length-1][fld];
	lines.unshift(l);
	l.label="";
	l.class="current"
	
	if (Array.isArray(lines)) {
		lines.forEach(function(x) {
			var v = x[fld];
			if (v) {
				var y = map_y(v);
				new_svg_el("line",{x1:0,y1:y,x2:activewidth+axis,y2:y,class:"orderline "+x.class},svg);
				new_svg_el("text",{x:axis,y:y,class:"textorderline "+x.class},svg).appendChild(document.createTextNode(x.label + " "+adjNum(v)));
			}
		});
	}
	
};
}