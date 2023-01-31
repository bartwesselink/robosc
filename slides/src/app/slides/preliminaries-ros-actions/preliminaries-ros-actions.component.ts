import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-ros-actions',
  templateUrl: './preliminaries-ros-actions.component.html',
  styleUrls: ['./preliminaries-ros-actions.component.scss']
})
export class PreliminariesRosActionsComponent extends AbstractSlide implements OnInit {
  max: number = 4;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
