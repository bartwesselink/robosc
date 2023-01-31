import { Component, ContentChild, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';
import { SlideItem } from 'src/app/models/slide-item';

@Component({
  selector: 'app-overview-idea',
  templateUrl: './overview-idea.component.html',
  styleUrls: ['./overview-idea.component.scss']
})
export class OverviewIdeaComponent extends AbstractSlide implements OnInit {
  max: number = 9;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
