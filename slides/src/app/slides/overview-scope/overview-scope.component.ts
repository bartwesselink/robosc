import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-overview-scope',
  templateUrl: './overview-scope.component.html',
  styleUrls: ['./overview-scope.component.scss']
})
export class OverviewScopeComponent extends AbstractSlide implements OnInit {
  max: number = 6;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
