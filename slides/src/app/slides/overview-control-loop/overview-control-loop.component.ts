import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-overview-control-loop',
  templateUrl: './overview-control-loop.component.html',
  styleUrls: ['./overview-control-loop.component.scss']
})
export class OverviewControlLoopComponent extends AbstractSlide implements OnInit {
  max: number = 5;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
