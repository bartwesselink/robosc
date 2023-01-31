import { Component, ContentChild, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';
import { SlideItem } from 'src/app/models/slide-item';

@Component({
  selector: 'app-introduction-demonstration-robot',
  templateUrl: './introduction-demonstration-robot.component.html',
  styleUrls: ['./introduction-demonstration-robot.component.scss']
})
export class IntroductionDemonstrationRobotComponent extends AbstractSlide implements OnInit, SlideItem {
  max: number = 9;
  @ViewChild(SlideComponent) slide: SlideComponent;


  constructor() { super() }

  ngOnInit(): void {
  }

}
