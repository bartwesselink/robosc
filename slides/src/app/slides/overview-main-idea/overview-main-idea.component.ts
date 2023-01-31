import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-overview-main-idea',
  templateUrl: './overview-main-idea.component.html',
  styleUrls: ['./overview-main-idea.component.scss']
})
export class OverviewMainIdeaComponent extends AbstractSlide implements OnInit {
  max: number = 9;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
