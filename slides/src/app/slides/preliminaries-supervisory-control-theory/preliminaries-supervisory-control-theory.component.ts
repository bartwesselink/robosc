import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-supervisory-control-theory',
  templateUrl: './preliminaries-supervisory-control-theory.component.html',
  styleUrls: ['./preliminaries-supervisory-control-theory.component.scss']
})
export class PreliminariesSupervisoryControlTheoryComponent extends AbstractSlide implements OnInit {
  max: number = 6;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
