import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-evaluation-scenarios',
  templateUrl: './evaluation-scenarios.component.html',
  styleUrls: ['./evaluation-scenarios.component.scss']
})
export class EvaluationScenariosComponent extends AbstractSlide implements OnInit {
  max: number = 10;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
