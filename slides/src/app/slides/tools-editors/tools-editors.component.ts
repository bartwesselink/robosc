import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-tools-editors',
  templateUrl: './tools-editors.component.html',
  styleUrls: ['./tools-editors.component.scss']
})
export class ToolsEditorsComponent extends AbstractSlide implements OnInit {
  max: number = 5;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
