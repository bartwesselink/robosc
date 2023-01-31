import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-mde',
  templateUrl: './preliminaries-mde.component.html',
  styleUrls: ['./preliminaries-mde.component.scss']
})
export class PreliminariesMdeComponent extends AbstractSlide implements OnInit {
  max: number = 9;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
