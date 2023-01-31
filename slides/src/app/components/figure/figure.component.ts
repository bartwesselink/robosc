import { Component, Input, OnInit } from '@angular/core';
import { FigureService } from 'src/app/services/figure.service';

@Component({
  selector: 'app-figure',
  templateUrl: './figure.component.html',
  styleUrls: ['./figure.component.scss']
})
export class FigureComponent implements OnInit {
  public number: number = this.figureService.obtainNumber();
  @Input() public caption: string;
  @Input() public offset = 0;
  @Input() public step: number|undefined = undefined;
  @Input() public current: number|undefined = undefined;

  constructor(private figureService: FigureService) { }

  ngOnInit(): void {
  }

  get visible() {
    return this.step === undefined || this.step <= this.current;
  }
}
