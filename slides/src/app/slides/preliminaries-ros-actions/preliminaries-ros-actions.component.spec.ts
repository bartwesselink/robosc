import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesRosActionsComponent } from './preliminaries-ros-actions.component';

describe('PreliminariesRosActionsComponent', () => {
  let component: PreliminariesRosActionsComponent;
  let fixture: ComponentFixture<PreliminariesRosActionsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesRosActionsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesRosActionsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
