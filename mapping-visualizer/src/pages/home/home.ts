import { Component, ViewChild, ElementRef } from '@angular/core';
import { Chart } from 'chart.js';

@Component({
  selector: 'page-home',
  templateUrl: 'home.html'
})
export class HomePage {

  public data: number[] = [20, 15, 20, 20,20, 15, 20, 20,20, 15];
  public labelsArr: int[] = Array.from(Array(10).keys());
  public coptions: any;
  public myChart: any;
  public ctx: any;

  @ViewChild('myChart', {read: ElementRef}) public ctxEl: ElementRef;

  public constructor() {
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
