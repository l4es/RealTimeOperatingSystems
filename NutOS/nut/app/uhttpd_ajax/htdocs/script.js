var req = GetRequestObject();
var wdog = 1;

window.onload = main();

function GetRequestObject()
{
  if(window.XMLHttpRequest) {
    return new XMLHttpRequest();
  }
  else if (window.ActiveXObject) {
    try {
      return new ActiveXObject("Msxml2.XMLHTTP");
    } catch (e) {
      try {
        return new ActiveXObject("Microsoft.XMLHTTP");
      } catch (e) {
      }
    }
  } 
  alert('Ajax not available on this browser');
}

function RequestData()
{
  if (req.readyState == 0) {
    wdog++;
    req.open('GET', 'clock.cgi', true);
    req.onreadystatechange = DisplayTime;
    req.send(null);
  } else {
    req.onreadystatechange = function () { };
    req.abort();
    setTimeout("RequestData();", 100);
  }
}

function Watchdog() {
  if (wdog != 0) {
    wdog = 0;
  } else {
    req.onreadystatechange = function () { };
    req.abort();
    setTimeout("RequestData();", 100);
  }
  setTimeout("Watchdog();", 5000);
}

function DisplayTime()
{
  if(req.readyState == 4) {
    var response = eval('(' + req.responseText + ')');
    document.getElementById('time').innerHTML = response.time;
    RequestData();
  }
}

function main() {
  RequestData();
  Watchdog();
}
