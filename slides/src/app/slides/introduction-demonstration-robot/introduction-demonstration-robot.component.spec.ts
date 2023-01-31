import { ComponentFixture, TestBed } from '@angular/core/testing';

import { IntroductionDemonstrationRobotComponent } from './introduction-demonstration-robot.component';

describe('IntroductionDemonstrationRobotComponent', () => {
  let component: IntroductionDemonstrationRobotComponent;
  let fixture: ComponentFixture<IntroductionDemonstrationRobotComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ IntroductionDemonstrationRobotComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(IntroductionDemonstrationRobotComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
