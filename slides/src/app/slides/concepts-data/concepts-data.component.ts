import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-concepts-data',
  templateUrl: './concepts-data.component.html',
  styleUrls: ['./concepts-data.component.scss']
})
export class ConceptsDataComponent extends AbstractSlide implements OnInit {
  max: number = 12;
  @ViewChild(SlideComponent) slide: SlideComponent;

  code = `robot PresentationRobot {
    ...

    _highlight8_ interface geometry use Twist from geometry_msgs _/highlight_

    _highlight9_ datatype object Vector3 {
    x: double
    y: double
    z: double
}

datatype object Twist {
    angular: Vector3
    linear: Vector3
}  _/highlight_

    _highlight10_ datatype enum Safety from integer to {
    value > 10 -> safe_value
    default -> unsafe_value
}  _/highlight_

    _highlight11_ provide forward with { linear: { x: 0.4 } } if Line.no_lines
provide forward with { angular: { z: 0.2 } } if Line.line_left
provide forward with { angular: { z: -0.2 } } if Line.line_right

provide stop with { linear: { x: 0.0 }, angular: { z: 0.0 } }  _/highlight_

    ...
}`

  constructor() { super() }

  ngOnInit(): void {
  }

}
