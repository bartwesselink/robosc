import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConclusionContributionsComponent } from './conclusion-contributions.component';

describe('ConclusionContributionsComponent', () => {
  let component: ConclusionContributionsComponent;
  let fixture: ComponentFixture<ConclusionContributionsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConclusionContributionsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConclusionContributionsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
