/////////////Ajax Requests////////////
$(document).ready(function () {
  // Fetch the initial table
  refreshTable();
  // Fetch the initial Map
  refreshMap();

  // Fetch the initial Chart
  refreshChart();
  // Fetch every 5 second
  setInterval(refreshChart, 10000);

  // Fetch every 1 second
  setInterval(refreshTable, 15000);
  setInterval(refreshMap, 15000);
});

function myFunction() {
    
  var input, filter, table, tr, td, i, txtValue;
  input = document.getElementById("myInput");
  console.log(input);
  filter = input.value;
  table = document.getElementById("tableContent");
  tr = table.getElementsByTagName("tr");

  // Loop through all table rows, and hide those who don't match the search query
  for (i = 0; i < tr.length; i++) {
      td = tr[i].getElementsByTagName("td")[0];
      if (td) {
      txtValue = td.textContent || td.innerText;
      if (txtValue.toUpperCase().indexOf(filter) > -1) {
          tr[i].style.display = "";
      } else {
          tr[i].style.display = "none";
      }
      }
  }
}

google.charts.load("current", { packages: ["corechart"] });
google.charts.setOnLoadCallback(refreshChart);

function refreshTable() {
  // var trHTML = '';

  $.getJSON(
    "https://spreadsheets.google.com/feeds/list/14PQ_c-u9KFF3aCm3LoCNs8qCKOw0OeqgZKgvQ9Gf2ZM/1/public/full?alt=json",
    function (data) {
      var trHTML = "";

      for (var i = 0; i < data.feed.entry.length; ++i) {

        trHTML +=
          "<tr><td>" +
          data.feed.entry[i].gsx$orderid.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$item.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$priority.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$city.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$dispatchstatus.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$shippedstatus.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$orderdateandtime.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$dispatchdateandtime.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$shippeddateandtime.$t +
          "</td><td>" +
          data.feed.entry[i].gsx$timetaken.$t +
          "</td></tr>";
      }

      console.log(trHTML);
      $("#tableContent").html(trHTML);
      var trHTML = "";
    }
  );
}

function refreshMap() {
  var container = L.DomUtil.get("map");

  if (container != null) {
    container._leaflet_id = null;
  }

  var map = L.map("map").setView([20.5837, 78.9629], 4);
  var jsonDataObject = [];

  $.getJSON(
    "https://spreadsheets.google.com/feeds/list/14PQ_c-u9KFF3aCm3LoCNs8qCKOw0OeqgZKgvQ9Gf2ZM/1/public/full?alt=json",
    function (data) {
      for (var i = 0; i < data.feed.entry.length; ++i) {
        var json_data = {
          City: data.feed.entry[i].gsx$city.$t,
          OderID: data.feed.entry[i].gsx$orderid.$t,
          Item: data.feed.entry[i].gsx$item.$t,
          Latitude: parseFloat(data.feed.entry[i].gsx$latitude.$t),
          Longitude: parseFloat(data.feed.entry[i].gsx$longitude.$t),
          Dispatch_status: data.feed.entry[i].gsx$dispatchstatus.$t,
          Shipped_status: data.feed.entry[i].gsx$shippedstatus.$t,
        };
        jsonDataObject.push(json_data);

        for (var j = 0; j < jsonDataObject.length; j++) {
          var marker = L.marker(
            [
              parseFloat(jsonDataObject[j].Latitude),
              parseFloat(jsonDataObject[j].Longitude),
            ],
            {
              icon: set_marker_color(
                jsonDataObject[j].Dispatch_status,
                jsonDataObject[j].Shipped_status
              ),
            }
          );

          var customPopup;
          // specify popup options
          var customOptions = { className: "custom-popup" };
          marker.bindPopup(customPopup, customOptions, { autoClose: false });
          map.addLayer(marker);
          marker.on("click", onClick_Marker);
          // Attach the corresponding JSON data to your marker:
          marker.myJsonData = jsonDataObject[j];

          function onClick_Marker(e) {
            var marker = e.target;
            popup = L.popup()
              .setLatLng({ lat: 24.077, lng: 101.5559 })
              .setContent(
                "<center>" +
                  "<p>" +
                  marker.myJsonData.City +
                  "</p>" + "<p>" +
                  "Order ID :  " +
                  marker.myJsonData.OderID +
                  "</p>" +
                  " Item : " +
                  marker.myJsonData.Item +
                  "</center>"
              )
              .openOn(map);
          }
        }

        function set_marker_color(dis_status, ship_status) {
          var str1 =
            "https://github.com/pointhi/leaflet-color-markers/blob/master/img/marker-icon-";
          var str2 = ".png?raw=true";
          var str3 = "";

          if (dis_status == "YES" && ship_status == "YES") {
            str3 = str1.concat("green", str2);
          } else if (dis_status == "YES" && ship_status == "NO") {
            str3 = str1.concat("yellow", str2);
          } else {
            str3 = str1.concat("red", str2);
          }
          var colorIcon = new L.Icon({
            iconUrl: str3,
            shadowUrl:
              "https://github.com/pointhi/leaflet-color-markers/blob/master/img/marker-shadow.png?raw=true",
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            popupAnchor: [1, -34],
            shadowSize: [41, 41],
          });
          return colorIcon;
        }

        L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
          attribution:
            '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
        }).addTo(map);
      }
    }
  );
}

function refreshChart() {
  var jsonDataObject = [];
  var graph_arr = [["Order ID", "Time Taken", { role: "style" }]];
  var bar_color = [];
  $.getJSON(
    "https://spreadsheets.google.com/feeds/list/14PQ_c-u9KFF3aCm3LoCNs8qCKOw0OeqgZKgvQ9Gf2ZM/1/public/full?alt=json",
    function (data) {
      for (var i = 0; i < data.feed.entry.length; ++i) {
        var json_data = {
          OderID: data.feed.entry[i].gsx$orderid.$t,
          TimeTaken: parseFloat(data.feed.entry[i].gsx$timetaken.$t),
          Priority: data.feed.entry[i].gsx$priority.$t,
        };
        jsonDataObject.push(json_data);
      }
      // Setting color for the coloumns of graph according to priority of items
      for (var j in jsonDataObject) {
        if (jsonDataObject[j].Priority == "HP") {
          var color = "#ec4646";
        } else if (jsonDataObject[j].Priority == "MP") {
          var color = "#FFD700";
        } else if (jsonDataObject[j].Priority == "LP") {
          var color = "#21e6a3";
        }
        bar_color.push(color);
      }

      // Converting Json Object to JavaScript Array
      for (var j in jsonDataObject) {
        graph_arr.push([
          jsonDataObject[j].OderID,
          jsonDataObject[j].TimeTaken,
          bar_color[j],
        ]);
      }
      var graphArray_Final = google.visualization.arrayToDataTable(graph_arr);

      var data = new google.visualization.DataView(graphArray_Final);

      var options = {
        hAxis: { title: "Order ID" },
        vAxis: { title: "Time Taken (s)" },
        legend: { position: "none" },
      };
      var chart = new google.visualization.ColumnChart(
        document.getElementById("column_chart")
      );
      chart.draw(data, options);
    }
  );
}
