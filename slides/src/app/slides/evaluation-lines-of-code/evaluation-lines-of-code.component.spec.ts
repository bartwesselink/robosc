import { ComponentFixture, TestBed } from '@angular/core/testing';

import { EvaluationLinesOfCodeComponent } from './evaluation-lines-of-code.component';

describe('EvaluationLinesOfCodeComponent', () => {
  let component: EvaluationLinesOfCodeComponent;
  let fixture: ComponentFixture<EvaluationLinesOfCodeComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ EvaluationLinesOfCodeComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(EvaluationLinesOfCodeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
