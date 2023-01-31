import { Component, OnInit, ViewChild } from '@angular/core';
import { SlideComponent } from 'src/app/components/slide/slide.component';
import { AbstractSlide } from 'src/app/models/abstract-slide';

@Component({
  selector: 'app-concepts-communication',
  templateUrl: './concepts-communication.component.html',
  styleUrls: ['./concepts-communication.component.scss']
})
export class ConceptsCommunicationComponent extends AbstractSlide implements OnInit {
  max: number = 16;
  @ViewChild(SlideComponent) slide: SlideComponent;

  code = `robot PresentationRobot {
    ...
    component Motor {
        _highlight6_ incoming message forward with identifier: "/velocity",
            type: Twist links geometry
incoming message stop with identifier: "/velocity",
            type: Twist links geometry _/highlight_
    }
    component Distance {
        _highlight7_ outgoing message distance with type: Safety _/highlight_
        behaviour {
            ...
            _highlight8_ on response from distance do safety := value _/highlight_
            initial marked state unsafe {
                _highlight9_ transition if Distance.safety = safe_value goto safe _/highlight_
            }
            state safe {
                _highlight10_ transition if Distance.safety = unsafe_value goto unsafe _/highlight_
            }
        }
    }
    component Line {
        _highlight11_ outgoing message line_left with type: none
outgoing message line_right with type: none
outgoing message no_line_left with type: none
outgoing message no_line_right with type: none _/highlight_
        behaviour {
            initial marked state no_lines {
                _highlight12_ on response from line_left goto line_left
on response from line_right goto line_right _/highlight_
            }
            state line_left {
                _highlight13_ on response from line_right goto both_lines
on response from no_line_left goto no_lines _/highlight_
            }
            state line_right {
                _highlight14_ on response from line_left goto both_lines
on response from no_line_right goto no_lines _/highlight_
            }
            state both_lines {
                _highlight15_ on response from no_line_left goto line_right
on response from no_line_right goto line_left _/highlight_
            }
        }
    }
}`

  constructor() { super() }

  ngOnInit(): void {
  }

}
