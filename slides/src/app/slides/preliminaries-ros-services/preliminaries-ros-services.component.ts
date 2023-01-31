import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-ros-services',
  templateUrl: './preliminaries-ros-services.component.html',
  styleUrls: ['./preliminaries-ros-services.component.scss']
})
export class PreliminariesRosServicesComponent extends AbstractSlide implements OnInit {
  max: number = 5;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
