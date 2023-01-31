import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-ros-messages',
  templateUrl: './preliminaries-ros-messages.component.html',
  styleUrls: ['./preliminaries-ros-messages.component.scss']
})
export class PreliminariesRosMessagesComponent extends AbstractSlide implements OnInit {
  max: number = 4;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
