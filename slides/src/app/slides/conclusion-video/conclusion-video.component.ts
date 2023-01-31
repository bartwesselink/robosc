import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-conclusion-video',
  templateUrl: './conclusion-video.component.html',
  styleUrls: ['./conclusion-video.component.scss']
})
export class ConclusionVideoComponent extends AbstractSlide implements OnInit {
  max: number = 1;
  @ViewChild(SlideComponent) slide: SlideComponent;
  @ViewChild('video') video: ElementRef<HTMLVideoElement>;

  constructor() { super() }

  ngOnInit(): void {
  }

  next(): void {
    super.next();

    this.video.nativeElement.play().then(v => console.log('info', v));
  }
}
