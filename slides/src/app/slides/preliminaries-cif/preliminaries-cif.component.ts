import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-preliminaries-cif',
  templateUrl: './preliminaries-cif.component.html',
  styleUrls: ['./preliminaries-cif.component.scss']
})
export class PreliminariesCifComponent extends AbstractSlide implements OnInit {
  max: number = 6;
  @ViewChild(SlideComponent) slide: SlideComponent;

  constructor() { super() }

  ngOnInit(): void {
  }

}
