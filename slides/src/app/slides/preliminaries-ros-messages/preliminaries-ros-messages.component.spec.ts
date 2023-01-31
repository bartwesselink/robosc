import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesRosMessagesComponent } from './preliminaries-ros-messages.component';

describe('PreliminariesRosMessagesComponent', () => {
  let component: PreliminariesRosMessagesComponent;
  let fixture: ComponentFixture<PreliminariesRosMessagesComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesRosMessagesComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesRosMessagesComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
