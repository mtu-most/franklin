var widget;

function init ()
{
	widget = document.getElementById ('printfile');
	open_websocket ();
}

function open_websocket ()
{
	websocket = #WEBSOCKET#;
	websocket.onmessage = function (frame) { message (JSON.parse (frame.data)); }
	websocket.onopen = setup;
	websocket.onclose = function () { alert ('connection to server lost'); }
}

function send (data)
{
	websocket.send (JSON.stringify (data));
}

function setup ()
{
	setInterval (function () { send (['status', ''])}, 1000);
}

function message (data) {
	if (data[0] == 'status')
	{
		settext (document.getElementById ('btemp_read'), data[1][0]);
		settext (document.getElementById ('etemp_read'), data[1][1]);
	}
}

function settext (element, text)
{
	while (element.firstChild)
		element.removeChild (element.firstChild);
	element.appendChild (document.createTextNode (text));
}

function print () {
	if (widget.files.length < 1)
		return;
	var reader = new FileReader ();
	reader.onloadend = function (e) {
		if (e.target.readyState == FileReader.DONE) {
			send (['gcode', e.target.result]);
			send (['ping', '']);
		}
	}
	reader.readAsBinaryString (widget.files[0]);
}

function etemp ()
{
	send (['etemp', document.getElementById ('etemp').value]);
}

function btemp ()
{
	send (['btemp', document.getElementById ('btemp').value]);
}

function stop ()
{
	send (['stop', '']);
}

function resume ()
{
	send (['resume', '']);
}

function home ()
{
	send (['home', '']);
}

function homez ()
{
	send (['homez', '']);
}

function sleep ()
{
	send (['sleep', '']);
}

function feed ()
{
	send (['feed', document.getElementById ('feed').value]);
}
