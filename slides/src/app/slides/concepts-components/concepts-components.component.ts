import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-concepts-components',
  templateUrl: './concepts-components.component.html',
  styleUrls: ['./concepts-components.component.scss']
})
export class ConceptsComponentsComponent extends AbstractSlide implements OnInit {
  max: number = 8;
  @ViewChild(SlideComponent) slide: SlideComponent;

  code = `robot PresentationRobot {
    _highlight5_ component Motor { }  _/highlight_
    _highlight6_ component Distance {
  behaviour {
      variable safety: Safety = unsafe_value

      initial marked state unsafe {}
      state safe {}
  }
}  _/highlight_

    _highlight7_ component Line {
  behaviour {
    initial marked state no_lines {}
    state line_left {}
    state line_right {}
    state both_lines {}
  }
}  _/highlight_
}`

  constructor() { super() }

  ngOnInit(): void {
  }

}
