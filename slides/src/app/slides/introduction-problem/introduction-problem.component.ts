import { Component, ContentChild, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';
import { SlideItem } from 'src/app/models/slide-item';

@Component({
  selector: 'app-introduction-problem',
  templateUrl: './introduction-problem.component.html',
  styleUrls: ['./introduction-problem.component.scss']
})
export class IntroductionProblemComponent extends AbstractSlide implements OnInit {
  max: number = 7;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
