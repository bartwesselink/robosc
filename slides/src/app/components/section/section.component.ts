import { Component, ContentChildren, Input, OnInit, QueryList } from '@angular/core';
import { ComponentType } from 'src/app/models/component-type';
import { SlideItem } from 'src/app/models/slide-item';
import { VisibleComponent } from 'src/app/models/visible-component';
import { SlideComponent } from '../slide/slide.component';

@Component({
  selector: 'app-section',
  templateUrl: './section.component.html',
  styleUrls: ['./section.component.scss']
})
export class SectionComponent implements OnInit, VisibleComponent {
  @ContentChildren('slide') public slides: QueryList<SlideItem>;
  @Input() public number: number;
  @Input() public label: string;
  public active: boolean = false;
  public offsetX = 0;
  public offsetY = 0;
  public componentType = ComponentType.SECTION;
  public reset: number;

  constructor() { }

  ngOnInit(): void {
  }

  isCompleted() {
    return true;
  }
}
