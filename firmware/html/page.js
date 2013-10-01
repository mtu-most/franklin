var widget;

function init ()
{
	widget = document.getElementById ('printfile');
	open_websocket ();
}

function open_websocket ()
{
	if (window.hasOwnProperty ('MozWebSocket'))
	{
		// Old firefox.
		websocket = new MozWebSocket ('wss://HOSTNAME');
	}
	else
	{
		websocket = new WebSocket ('wss://HOSTNAME');
	}
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
}

function message (data) {
	alert (data);
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
