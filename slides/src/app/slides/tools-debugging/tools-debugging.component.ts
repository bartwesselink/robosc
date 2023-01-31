import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-tools-debugging',
  templateUrl: './tools-debugging.component.html',
  styleUrls: ['./tools-debugging.component.scss']
})
export class ToolsDebuggingComponent extends AbstractSlide implements OnInit {
  max: number = 4;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
