import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-concepts-requirements',
  templateUrl: './concepts-requirements.component.html',
  styleUrls: ['./concepts-requirements.component.scss']
})
export class ConceptsRequirementsComponent extends AbstractSlide implements OnInit {
  max: number = 11;
  @ViewChild(SlideComponent) slide: SlideComponent;

  code = `robot PresentationRobot {
    ...

    _highlight8_ requirement Line.both_lines disables forward _/highlight_
    _highlight9_ requirement forward needs Distance.safe _/highlight_

    _highlight10_ requirement stop needs Line.both_lines or Distance.unsafe _/highlight_

    ...
}`

  constructor() { super() }

  ngOnInit(): void {
  }

}
