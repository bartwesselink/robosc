import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-supervisor-idea',
  templateUrl: './supervisor-idea.component.html',
  styleUrls: ['./supervisor-idea.component.scss']
})
export class SupervisorIdeaComponent extends AbstractSlide implements OnInit {
  max: number = 6;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
