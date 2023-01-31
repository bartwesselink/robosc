import { ComponentFixture, TestBed } from '@angular/core/testing';

import { IntroductionProblemComponent } from './introduction-problem.component';

describe('IntroductionProblemComponent', () => {
  let component: IntroductionProblemComponent;
  let fixture: ComponentFixture<IntroductionProblemComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ IntroductionProblemComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(IntroductionProblemComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
