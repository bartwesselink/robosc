import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-supervisor-limitations',
  templateUrl: './supervisor-limitations.component.html',
  styleUrls: ['./supervisor-limitations.component.scss']
})
export class SupervisorLimitationsComponent extends AbstractSlide implements OnInit {
  max: number = 4;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
