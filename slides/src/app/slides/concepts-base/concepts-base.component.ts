import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-concepts-base',
  templateUrl: './concepts-base.component.html',
  styleUrls: ['./concepts-base.component.scss']
})
export class ConceptsBaseComponent extends AbstractSlide implements OnInit {
  max: number = 8;
  @ViewChild(SlideComponent) slide: SlideComponent;

  code = `robot PresentationRobot {
    _highlight7_ // Here we define the behaviour  _/highlight_
}`

  constructor() { super() }

  ngOnInit(): void {
  }

}
