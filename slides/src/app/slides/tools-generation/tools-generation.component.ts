import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-tools-generation',
  templateUrl: './tools-generation.component.html',
  styleUrls: ['./tools-generation.component.scss']
})
export class ToolsGenerationComponent extends AbstractSlide implements OnInit {
  max: number = 8;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
