import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-conclusion-contributions',
  templateUrl: './conclusion-contributions.component.html',
  styleUrls: ['./conclusion-contributions.component.scss']
})
export class ConclusionContributionsComponent extends AbstractSlide implements OnInit {
  max: number = 8;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
