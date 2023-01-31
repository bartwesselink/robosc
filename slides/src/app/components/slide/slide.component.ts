import { Component, Inject, OnInit } from '@angular/core';
import { ComponentType } from 'src/app/models/component-type';
import { VisibleComponent } from 'src/app/models/visible-component';
import { SectionComponent } from '../section/section.component';

@Component({
  selector: 'app-slide',
  templateUrl: './slide.component.html',
  styleUrls: ['./slide.component.scss']
})
export class SlideComponent implements OnInit, VisibleComponent {
  public active: boolean = false;
  public componentType = ComponentType.SLIDE;
  public reset: number;

  constructor() {}

  ngOnInit(): void {
  }

  isCompleted() {
    return true;
  }
}
