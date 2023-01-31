import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-ros',
  templateUrl: './preliminaries-ros.component.html',
  styleUrls: ['./preliminaries-ros.component.scss']
})
export class PreliminariesRosComponent extends AbstractSlide implements OnInit {
  max: number = 5;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
