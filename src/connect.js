
$(document).ready(function () {
  $("#connect_button_input").click(function () {
    connect();
  });
});

function connect() {
  var robotHost = $("#robotHost").val();
  console.log(robotHost);

  ros = new ROSLIB.Ros({
    url : 'ws://' + robotHost
  });

  $("#error").hide();
  $("#interface").hide();
  $("#spinner").show();

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $("#spinner").hide();
    $("#interface").show();
    initMap();
  });

  ros.on('error', function(error) {
    console.log('Could not connect to websocket server: ', error);
    $("#spinner").hide();
    $("#error").text("Could not connect to websocket server");
    $("#error").show();
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  var nameParam = new ROSLIB.Param({
    ros : ros,
    name : '/name'
  });

  nameParam.get(function (value) {
    robotName = value;
    console.log("Robot name: ", value);
    startHandlePallet();
  });

}
