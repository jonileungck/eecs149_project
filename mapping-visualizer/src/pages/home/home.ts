import { Component, ViewChild, ElementRef } from '@angular/core';
import { Platform } from 'ionic-angular';
import { Chart } from 'chart.js';
import { BLE } from '@ionic-native/ble';

@Component({
  selector: 'page-home',
  templateUrl: 'home.html'
})
export class HomePage {

  public data: number[] = [20, 15, 20, 20,20, 15, 20, 20,20, 15];
  public labelsArr: number[] = Array.from(Array(10).keys());
  public coptions: any;
  public myChart: any;
  public ctx: any;
  public chart: any;

  @ViewChild('myChart', {read: ElementRef}) public ctxEl: ElementRef;

  public constructor(public plt: Platform, private ble: BLE) {
    plt.ready().then(() => {
      console.log("Scanning devices..");
      ble.startScan([]).subscribe({
        next: function(device) {
          if(device.id == "C0:98:E5:49:00:01") {
            ble.stopScan();
            console.log("Found buckler!");
            console.log(JSON.stringify(device));
            console.log("Connecting to buckler..");
            ble.connect(device.id).subscribe({
              data: function(data){console.log(data)},
              error: function(e) {
                console.log(JSON.stringify(e));
              }
            });
            //ble.read()
          }
        },
        error: function(e) {
          console.log(JSON.stringify(e));
        }
      });
      // let scanObserver = ble.scan([], 5);
      // scanObserver.subscribe({
      //   next: device => {
      //     if(device.id == this.bucklerMac) {
      //       console.log('Connecting to device: ' + device.id);
      //       // let connectObserver = ble.connect(device.id);
      //       // connectObserver.subscribe({
      //       //
      //       // });
      //       ble.read(device.id, "", "",
      //         function(data){
      //             console.log("Hooray we have data"+JSON.stringify(data));
      //             alert("Successfully read data from device."+JSON.stringify(data));
      //         },
      //         function(failure){
      //             alert("Failed to read characteristic from device.");
      //         }
      //       );
      //     }
      //   },
      // });
    });
  }

  public ngAfterViewInit(): any {
    this.coptions = {
      type: 'radar',
      data: {
          labels: this.labelsArr,
          datasets: [{
              label: 'Mapping',
              data: this.data,
              backgroundColor: 'rgba(255, 99, 132, 0.2)',
              borderColor: 'rgb(255, 99, 132)'
          }]
      },
      options: {
        scale: {
          ticks: {
            beginAtZero: true
          }
        }
      }
    };

    this.ctx = this.ctxEl.nativeElement.getContext('2d');
    console.log(this.coptions);
    console.log(this.ctx);

    this.chart = new Chart(this.ctx, this.coptions);
  }

}

// let data = [20, 15, 20, 20,20, 15, 20, 20,20, 15];
// const labelsArr = Array.from(Array(10).keys());
//
// const ctx = document.getElementById("myChart");
// const myChart = new Chart(ctx, {
//   type: 'radar',
//   data: {
//       labels: labelsArr,
//       datasets: [{
//           label: 'Mapping',
//           data,
//           backgroundColor: 'rgba(255, 99, 132, 0.2)',
//           borderColor: 'rgb(255, 99, 132)'
//       }]
//   },
//   options: {
//     scale: {
//       ticks: {
//         beginAtZero: true
//       }
//     }
//   }
// });
