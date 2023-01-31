import { Component, OnInit, ViewChild } from '@angular/core';
import { ChartConfiguration } from 'chart.js';
import { BaseChartDirective } from 'ng2-charts';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-evaluation-lines-of-code',
  templateUrl: './evaluation-lines-of-code.component.html',
  styleUrls: ['./evaluation-lines-of-code.component.scss']
})
export class EvaluationLinesOfCodeComponent extends AbstractSlide implements OnInit {
  max: number = 7;
  @ViewChild(SlideComponent) slide: SlideComponent;
  @ViewChild(BaseChartDirective) chart: BaseChartDirective;

  public barChartLegend = true;
  public barChartPlugins = [];

  public barChartData: ChartConfiguration<'bar'>['data'] = {
    labels: [ 'Line Follower (controller)', 'Line Follower (supervised)', 'Maze Solver', 'Object Finder', 'Obstacle Navigation', 'Person Follower', 'Push Ball into Goal', 'Simple Navigation' ],
    datasets: [
      { barPercentage: undefined, barThickness: 15, data: [0, 0, 0, 0, 0, 0, 0, 0], label: 'DSL', backgroundColor: '#e74c3c', borderRadius: 5 },
      { barPercentage: undefined, barThickness: 15, data: [0, 0, 0, 0, 0, 0, 0, 0], label: 'CIF', backgroundColor: '#3498db', borderRadius: 5 },
      { barPercentage: undefined, barThickness: 15, data: [0, 0, 0, 0, 0, 0, 0, 0], label: 'ROS1', backgroundColor: '#f1c40f', borderRadius: 5 },
      { barPercentage: undefined, barThickness: 15, data: [0, 0, 0, 0, 0, 0, 0, 0], label: 'ROS2', backgroundColor: '#2ecc71', borderRadius: 5 },
    ],
  };

  public barChartOptions: ChartConfiguration<'bar'>['options'] = {
    responsive: false,
    indexAxis: 'y',
    font: {
      family: '"Fira Sans", sans-serif',
    },
    hover: {mode: null},
    plugins: {
        tooltip: {
          enabled: false,
        },
        legend: {
            labels: {
                font: {
                  family: '"Fira Sans", sans-serif',
                }
            }
        }
    },
    scales: {
      x: {
          min: 0,
          max: 1600,
          grid: {
              display: false
          },
          ticks: {
              font: {
                family: '"Fira Sans", sans-serif',
                size: 14,
              },
          },
      },
      y: {
        grid: {
            display: false
        },
        ticks: {
            font: {
              family: '"Fira Sans", sans-serif',
              size: 14,
            },
          },
      }
  }
  };

  constructor() { super() }

  ngOnInit(): void {
  }

  next(): void {
    super.next();
    this.checkStep();
  }

  prev(): void {
    super.prev();
    this.checkStep();
  }

  checkStep() {
    const allData = [
      [ 23, 36, 74, 90, 59, 60, 87, 59 ],
      [ 102, 87, 190, 239, 127, 130, 303, 127 ],
      [ 1245, 0, 1292, 1371, 1161, 1160, 1414, 1161 ],
      [ 1243, 1222, 1287, 1367, 1157, 1156, 1409, 1157 ],
    ];
    const empty = [0, 0, 0, 0, 0, 0, 0, 0];

    if (this.step === 1) {
      this.barChartData.datasets[0].data = allData[0];
      this.barChartData.datasets[1].data = [...empty];
      this.chart.update();
    } else if (this.step === 2) {
      this.barChartData.datasets[1].data = allData[1];
      this.barChartData.datasets[2].data = [...empty];
      this.chart.update();
    } else if (this.step === 3) {
      this.barChartData.datasets[2].data = allData[2];
      this.barChartData.datasets[3].data = [...empty];
      this.chart.update();
    } else if (this.step === 4) {
      this.barChartData.datasets[3].data = allData[3];
      this.chart.update();
    } else if (this.step === 0) {
      this.barChartData.datasets[0].data = [...empty];
      this.chart.update();
    }
  }
}
